
#ifndef _AMCL_SENSOR_H_
#define _AMCL_SENSOR_H_

#include "../ParticleFilter/pf.h"

namespace NS_Location
{

// Forward declarations
class AMCLSensorData;


// Base class for all AMCL sensors
class AMCLSensor
{
  // Default constructor
  public: AMCLSensor();
         
  // Default destructor
  public: virtual ~AMCLSensor();

  // Update the filter based on the action model.  Returns true if the filter
  // has been updated.
  public: virtual bool UpdateAction(pf_t *pf, AMCLSensorData *data);

  // Initialize the filter based on the sensor model.  Returns true if the
  // filter has been initialized.
  public: virtual bool InitSensor(pf_t *pf, AMCLSensorData *data);

  // Update the filter based on the sensor model.  Returns true if the
  // filter has been updated.
  public: virtual bool UpdateSensor(pf_t *pf, AMCLSensorData *data);

  // Flag is true if this is the action sensor
  public: bool is_action;

  // Action pose (action sensors only)
  public: pf_vector_t pose;

  // AMCL Base
  //protected: AdaptiveMCL & AMCL;

#ifdef INCLUDE_RTKGUI
  // Setup the GUI
  public: virtual void SetupGUI(rtk_canvas_t *canvas, rtk_fig_t *robot_fig);

  // Finalize the GUI
  public: virtual void ShutdownGUI(rtk_canvas_t *canvas, rtk_fig_t *robot_fig);

  // Draw sensor data
  public: virtual void UpdateGUI(rtk_canvas_t *canvas, rtk_fig_t *robot_fig, AMCLSensorData *data);
#endif
};



// Base class for all AMCL sensor measurements
class AMCLSensorData
{
  // Pointer to sensor that generated the data
  public: AMCLSensor *sensor;
          virtual ~AMCLSensorData() {}

  // Data timestamp
  public: double time;
};

}

#endif
