//
// C++ Interface: multitracker
//
// Description:
//
// Author: Nicola Bellotto <nbellotto@lincoln.ac.uk>, (C) 2011
//
// Copyright: See COPYING file that comes with this distribution
//
#ifndef MULTITRACKER_H
#define MULTITRACKER_H

#include "bayes_tracking/BayesFilter/bayesFlt.hpp"
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <boost/numeric/ublas/io.hpp>
#include <bayes_tracking/associationmatrix.h>
#include <bayes_tracking/jpda.h>
#include <bayes_tracking/models.h>
#include <float.h>


namespace MTRKYaw
{

struct observation_t
{
  FM::Vec vec;
  double time;
  double confidence;
  string flag;
  string name;
  // constructors
  observation_t() : vec(Empty), time(0.), confidence(1.0) {}
  observation_t(FM::Vec v) : vec(v), time(0.), confidence(1.0) {}
  observation_t(FM::Vec v, double t) : vec(v), time(t), confidence(1.0) {}
  observation_t(FM::Vec v, double t, double c) : vec(v), time(t), confidence(c)
  {
  }
  observation_t(FM::Vec v, double t, double c, string f)
      : vec(v), time(t), confidence(c), flag(f)
  {
  }
  observation_t(FM::Vec v, double t, double c, string f, string n)
      : vec(v), time(t), confidence(c), flag(f), name(n)
  {
  }
};

typedef std::vector<observation_t> sequence_t;

typedef enum
{
  NN,
  /*JPDA,*/ NNJPDA
} association_t;

typedef enum
{
  CARTESIAN,
  POLAR,
  CARTESIAN3D,
  CARTESIAN3DYaw
} observ_model_t;

// to be defined by user
// rule to detect lost track
template <class FilterType>
bool isLost(const FilterType* filter, double stdLimit, int *lost_filter_seq_count, unsigned int seqSize)
{
  // ROS_INFO("var_x: %f, var_y: %f",filter->X(0,0), filter->X(2,2));
  // track lost if var(x)+var(y) > stdLimit^2
  // only consider position, ignore yaw
  if (filter->X(0, 0) + filter->X(2, 2)> Models::sqr(stdLimit))
  {
    if(*lost_filter_seq_count > seqSize) return true;
    *lost_filter_seq_count += 1;
  }
  return false;
}

// rule to create new track
template <class FilterType>
bool initialize(FilterType*& filter, sequence_t& obsvSeq,
                observ_model_t om_flag)
{
  assert(obsvSeq.size());

  if (om_flag == CARTESIAN)
  {
    double dt = obsvSeq.back().time - obsvSeq.front().time;
    if (dt == 0)
      return false;
    // assert(dt); // dt must not be null

    FM::Vec v((obsvSeq.back().vec - obsvSeq.front().vec) / dt);
    FM::Vec x(4);
    FM::SymMatrix X(4, 4);

    x[0] = obsvSeq.back().vec[0];
    x[1] = v[0];
    x[2] = obsvSeq.back().vec[1];
    x[3] = v[1];
    X.clear();
    X(0, 0) = Models::sqr(0.2);
    X(1, 1) = Models::sqr(1.0);
    X(2, 2) = Models::sqr(0.2);
    X(3, 3) = Models::sqr(1.0);

    filter = new FilterType(4);
    filter->init(x, X);
  }

  if (om_flag == CARTESIAN3D || om_flag == CARTESIAN3DYaw)
  {
    double dt = obsvSeq.back().time - obsvSeq.front().time;
    if (dt == 0)
      return false;
    // assert(dt); // dt must not be null

    FM::Vec v((obsvSeq.back().vec - obsvSeq.front().vec) / dt);
    FM::Vec x(6);
    FM::SymMatrix X(6, 6);

    x[0] = obsvSeq.back().vec[0];
    x[1] = v[0];
    x[2] = obsvSeq.back().vec[1];
    x[3] = v[1];
    x[4] = obsvSeq.back().vec[2];
    x[5] = v[2];
    X.clear();
    X(0, 0) = Models::sqr(0.2);
    X(1, 1) = Models::sqr(1.0);
    X(2, 2) = Models::sqr(0.2);
    X(3, 3) = Models::sqr(1.0);
    X(4, 4) = Models::sqr(0.2);
    X(5, 5) = Models::sqr(1.0);

    filter = new FilterType(6);
    filter->init(x, X);
  }

  if (om_flag == POLAR)
  {
    double dt = obsvSeq.back().time - obsvSeq.front().time;
    if (dt == 0)
      return false;
    // assert(dt); // dt must not be null
    double x2 = obsvSeq.back().vec[1] * cos(obsvSeq.back().vec[0]);
    double y2 = obsvSeq.back().vec[1] * sin(obsvSeq.back().vec[0]);
    double x1 = obsvSeq.front().vec[1] * cos(obsvSeq.front().vec[0]);
    double y1 = obsvSeq.front().vec[1] * sin(obsvSeq.front().vec[0]);

    FM::Vec x(4);
    FM::SymMatrix X(4, 4);

    x[0] = x2;
    x[1] = (x2 - x1) / dt;
    x[2] = y2;
    x[3] = (y2 - y1) / dt;
    X.clear();
    X(0, 0) = Models::sqr(0.5);
    X(1, 1) = Models::sqr(1.5);
    X(2, 2) = Models::sqr(0.5);
    X(3, 3) = Models::sqr(1.5);

    filter = new FilterType(4);
    filter->init(x, X);
  }

  return true;
}

/**
   @author Nicola Bellotto <nick@robots.ox.ac.uk>
*/
template <class FilterType, int xSize> class MultiTracker
{
public:
  typedef struct
  {
    unsigned long id;
    FilterType* filter;
  } filter_t;

private:
  Float detS = 0.;

  double mahalanobis2(const FM::Vec& s,
                                        const FM::SymMatrix& Si)
  {
      return sqrt(inner_prod(trans(s), prod(Si, s)));  // sqrt(s' * Si * s)
  }

  double correlation_log2(const FM::Vec& s, const FM::SymMatrix& Si)
  {
      // s' * Si * s + ln|S|
      return inner_prod(trans(s), prod(Si, s)) + log(detS);
  }

  std::vector<filter_t> m_filters;
  int m_filterNum;
  sequence_t m_observations;        // observations
  std::vector<size_t> m_unmatched;  // unmatched observations
  std::map<int, int> m_assignments; // assignment < observation, target >
  std::map<long, int> m_lost_filter_seq_count; // lost_filter <lost_filter id, seq_count>
  std::vector<sequence_t>
      m_sequences; // vector of unmatched observation sequences

public:
  /**
   * Constructor
   */
  MultiTracker() { m_filterNum = 0; }

  /**
   * Destructor
   */
  ~MultiTracker()
  {
    typename std::vector<filter_t>::iterator fi, fiEnd = m_filters.end();
    for (fi = m_filters.begin(); fi != fiEnd; fi++)
      delete fi->filter;
  }

  /**
   * Add a new observation
   * @param z Observation vector
   * @param time Timestamp
   * @param flag Additional flags
   * @param name Detector name
   */
  void addObservation(const FM::Vec& z, double time, double confidence, string flag = "",
                      string name = "")
  {
    m_observations.push_back(observation_t(z, time, confidence, flag, name));
  }

  /**
   * Remove observations
   */
  void cleanup()
  {
    // clean current vectors
    m_observations.clear();
    m_assignments.clear();
  }

  /**
   * Size of the multitracker
   * @return Current number of filters
   */
  int size() { return m_filters.size(); }

  /**
   * Return a particular filter of the multitracker
   * @param i Index of the filter
   * @return Reference to the filter
   */
  const filter_t& operator[](int i) { return m_filters[i]; }

  /**
   * Perform prediction step for all the current filters
   * @param pm Prediction model
   */
  template <class PredictionModelType> void predict(PredictionModelType& pm)
  {
    typename std::vector<filter_t>::iterator fi, fiEnd = m_filters.end();
    for (fi = m_filters.begin(); fi != fiEnd; fi++)
    {
      fi->filter->predict(pm);
    }
  }
  /**
   * Perform data association and update step for all the current filters,
   * create new ones and remove those which are no more necessary
   * @param om Observation model
   * @param om_flag Observation model flag (CARTESIAN or POLAR)
   * @param alg Data association algorithm (NN or NNJPDA)
   * @param seqSize Minimum number of observations necessary for new track
   * creation
   * @param seqTime Minimum interval between observations for new track creation
   * @param stdLimit Upper limit for the standard deviation of the estimated
   * position
   */
  template <class ObservationModelType>
  void process(ObservationModelType& om, observ_model_t om_flag = CARTESIAN,
               association_t alg = NN, unsigned int create_seqSize = 5,
               double seqTime = 0.2, unsigned int prune_seqSize = 5, double stdLimit = 1.0)
  {
    // data association
    if (dataAssociation(om, alg))
    {
      // update
      observe(om);
    }
    pruneTracks(stdLimit, prune_seqSize);
    
    if (m_observations.size())
      createTracks(om, om_flag, create_seqSize, seqTime);
    // finished
    // cleanup();
  }

  /**
   * Returns the vector of indexes of unmatched observations
   */
  const std::vector<size_t>* getUnmatched() { return &m_unmatched; }

  const std::map<int, int>& getAssignments() { return m_assignments; }

  /**
   * Print state and covariance of all the current filters
   */
  void print()
  {
    int i = 0;
    typename std::vector<filter_t>::iterator fi, fiEnd = m_filters.end();
    for (fi = m_filters.begin(); fi != fiEnd; fi++)
    {
      cout << "Filter[" << i++ << "]\n\tx = " << fi->filter->x
           << "\n\tX = " << fi->filter->X << endl;
    }
  }

private:
  void addFilter(const FM::Vec& initState, const FM::SymMatrix& initCov)
  {
    FilterType* filter = new FilterType(xSize);
    filter.init(initState, initCov);
    addFilter(filter);
  }

  void addFilter(FilterType* filter)
  {
    filter_t f = {(unsigned long)m_filterNum++, filter};
    m_filters.push_back(f);
  }

  template <class ObservationModelType>
  bool dataAssociation(ObservationModelType& om, association_t alg = NN)
  {
    const size_t M = m_observations.size(), N = m_filters.size();

    if (M == 0) // no observation, do nothing
      return false;

    if (N != 0)
    { // observations and tracks, associate
      jpda::JPDA* jpda;
      vector<size_t>
          znum; // this would contain the number of observations for each sensor
      if (alg == NNJPDA)
      {                    /// NNJPDA data association (one sensor)
        znum.push_back(M); // only one in this case
        jpda = new jpda::JPDA(znum, N);
      }

      AssociationMatrix amat(M, N);
      int dim = om.z_size;
      Vec zp(dim), s(dim);
      SymMatrix Zp(dim, dim), S(dim, dim), Si(dim, dim);
      for (int j = 0; j < N; j++)
      {
        m_filters[j].filter->predict_observation(om, zp, Zp);
        S = Zp + om.Z; // H*P*H' + R

        Float rcond = UdUinversePD(Si, detS, S);  // Si = inv(S)
        Numerical_rcond rclimit;
        rclimit.check_PD(rcond, "S not PD in dataAssociation(...)");

        for (int i = 0; i < M; i++)
        {
          s = zp - m_observations[i].vec;
          om.normalise(s, zp);

          try
          {
            if (mahalanobis2(s, Si) > AM::gate(s.size()))
            {
              amat[i][j] = DBL_MAX; // gating
            }
            else
            {
              amat[i][j] = correlation_log2(s, Si);
              if (alg == NNJPDA)
              {
                jpda->Omega[0][i][j + 1] = true;
                jpda->Lambda[0][i][j + 1] = jpda::logGauss(s, S);
              }
            }
          }
          catch (Bayesian_filter::Filter_exception& e)
          {
            cerr << "###### Exception in AssociationMatrix #####\n";
            cerr << "Message: " << e.what() << endl;
            amat[i][j] = DBL_MAX; // set to maximum
          }
        }
      }

      if (alg == NN)
      { /// NN data association
        amat.computeNN(CORRELATION_LOG);
        // record unmatched observations for possible candidates creation
        m_unmatched = amat.URow;
        // data assignment
        for (size_t n = 0; n < amat.NN.size(); n++)
        {
          m_assignments.insert(std::make_pair(amat.NN[n].row, amat.NN[n].col));
        }
      }
      else if (alg == NNJPDA)
      { /// NNJPDA data association (one sensor)
        // compute associations
        jpda->getAssociations();
        jpda->getProbabilities();
        vector<jpda::Association> association(znum.size());
        jpda->getMultiNNJPDA(association);
        // jpda->getMonoNNJPDA(association);
        // data assignment
        jpda::Association::iterator ai, aiEnd = association[0].end();
        for (ai = association[0].begin(); ai != aiEnd; ai++)
        {
          if (ai->t)
          { // not a clutter
            m_assignments.insert(std::make_pair(ai->z, ai->t - 1));
          }
          else
          { // add clutter to unmatched list
            m_unmatched.push_back(ai->z);
          }
        }
        delete jpda;
      }
      else
      {
        cerr << "###### Unknown association algorithm: " << alg << " #####\n";
        return false;
      }

      return true;
    }

    for (int i = 0; i < M; i++) // simply record unmatched
      m_unmatched.push_back(i);

    return false;
  }

  void pruneTracks(double stdLimit, unsigned int seqSize)
  {
    // remove lost tracks
    typename std::vector<filter_t>::iterator fi = m_filters.begin(),
                                             fiEnd = m_filters.end();
    while (fi != fiEnd)
    {
      int *lost_filter_seq_count = &m_lost_filter_seq_count[fi->id];
      if (isLost(fi->filter, stdLimit, lost_filter_seq_count, seqSize))
      {
        delete fi->filter;
        fi = m_filters.erase(fi);
        fiEnd = m_filters.end();
      }
      else
      {
        fi++;
      }
    }
  }

  // seqSize = Minimum number of unmatched observations to create new track
  // hypothesis
  // seqTime = Maximum time interval between these observations
  template <class ObservationModelType>
  void createTracks(ObservationModelType& om, observ_model_t om_flag,
                    unsigned int seqSize,  double seqTime)
  {
    // create new tracks from unmatched observations
    std::vector<size_t>::iterator ui = m_unmatched.begin();
    while (ui != m_unmatched.end())
    {
      //const unsigned int adaptive_seqSize = seqSize * (1. - m_observations[*ui].confidence);
      std::vector<sequence_t>::iterator si = m_sequences.begin();
      bool matched = false;
      while (si != m_sequences.end())
      {
        if (m_observations[*ui].time - si->back().time > seqTime)
        { // erase old unmatched observations
          si = m_sequences.erase(si);
        }
        else if (AM::mahalanobis(m_observations[*ui].vec, om.Z, si->back().vec,
                                 om.Z) <= AM::gate(om.z_size))
        { 
          // in case of yaw diff pi
          if (m_observations[*ui].vec.size() == 3)
          {
            if ((m_observations[*ui].vec[2] - si->back().vec[2]) > 1.57)    // exceed pi/2
            {
              m_observations[*ui].vec[2] -= 3.1415926;  // ± pi correction
            }
            else if ((m_observations[*ui].vec[2] - si->back().vec[2]) < -1.57)
            {
              m_observations[*ui].vec[2] += 3.1415926; // ± pi correction
            }
          }
          // observation close to a previous one
          // add new track
          si->push_back(m_observations[*ui]);
          FilterType* filter;
          if (si->size() >= seqSize && initialize(filter, *si, om_flag))
          { // there's a minimum number of sequential observations
            addFilter(filter);
            m_assignments.insert(std::pair<int, int>(*ui, m_filterNum-1));
            // remove sequence
            si = m_sequences.erase(si);
            matched = true;
          }
          else
          {
            si++;
          }
        }
        else
        {
//          cout << "Vec(v1-v2):\n" << Vec(m_observations[*ui].vec-si->back().vec) << endl;
//          cout << "SymMatrix(R1+R2):\n" << SymMatrix(om.Z+om.Z) << endl;

//          cout << "mahalanobis: " << AM::mahalanobis(m_observations[*ui].vec, om.Z, si->back().vec,
//              om.Z) << endl;
          si++;
        }
      }
      if (matched)
      {
        // remove from unmatched list
        ui = m_unmatched.erase(ui);
      }
      else
      {
        ui++;
      }
    }
    // memorize remaining unmatched observations
    std::vector<size_t>::iterator uiEnd = m_unmatched.end();
    for (ui = m_unmatched.begin(); ui != uiEnd; ui++)
    {
      sequence_t s;
      s.push_back(m_observations[*ui]);
      m_sequences.push_back(s);
    }
    // reset vector of (indexes of) unmatched observations
    m_unmatched.clear();
  }

  template <class ObservationModelType> void observe(ObservationModelType& om)
  {
    typename std::map<int, int>::iterator ai, aiEnd = m_assignments.end();
    for (ai = m_assignments.begin(); ai != aiEnd; ai++)
    { 
      // cout << "Vec first:" << Vec(m_observations[ai->first].vec) << endl;
      // cout << "Vec second:" << m_filters[ai->second].filter->x[0] << " " << m_filters[ai->second].filter->x[2] << " " << m_filters[ai->second].filter->x[4] << endl;
      
      // in case of yaw diff pi
      if (m_observations[ai->first].vec.size() == 3)
      {
        if ((m_observations[ai->first].vec[2] - m_filters[ai->second].filter->x[4]) > 1.57)    // exceed pi/2
        {
          m_observations[ai->first].vec[2] -= 3.1415926;  // ± pi correction
        }
        else if ((m_observations[ai->first].vec[2] - m_filters[ai->second].filter->x[4]) < -1.57)
        {
          m_observations[ai->first].vec[2] += 3.1415926; // ± pi correction
        }
      }
      m_filters[ai->second].filter->observe(om, m_observations[ai->first].vec);
    }
  }
};
} // namespace MTRKYaw

#endif
