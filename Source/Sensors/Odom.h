
#ifndef _AMCL_ODOM_H_
#define _AMCL_ODOM_H_

#include "../ParticleFilter/pf_pdf.h"
#include "Sensor.h"

namespace NS_Location
{

typedef enum
{
  ODOM_MODEL_DIFF,
  ODOM_MODEL_OMNI,
  ODOM_MODEL_DIFF_CORRECTED,
  ODOM_MODEL_OMNI_CORRECTED
} odom_model_t;

// Odometric sensor data
class AMCLOdomData : public AMCLSensorData
{
  // Odometric pose
  public: pf_vector_t pose;

  // Change in odometric pose
  public: pf_vector_t delta;
};


// Odometric sensor model
class AMCLOdom : public AMCLSensor
{
  // Default constructor
  public: AMCLOdom();

  public: void SetModelDiff(double alpha1, 
                            double alpha2, 
                            double alpha3, 
                            double alpha4);

  public: void SetModelOmni(double alpha1, 
                            double alpha2, 
                            double alpha3, 
                            double alpha4,
                            double alpha5);

  public: void SetModel( odom_model_t type,
                         double alpha1,
                         double alpha2,
                         double alpha3,
                         double alpha4,
                         double alpha5 = 0 );

  // Update the filter based on the action model.  Returns true if the filter
  // has been updated.
  public: virtual bool UpdateAction(pf_t *pf, AMCLSensorData *data);

  // Current data timestamp
  private: double time;
  
  // Model type
  private: odom_model_t model_type;

  // Drift parameters
  private: double alpha1, alpha2, alpha3, alpha4, alpha5;
};


}

#endif
