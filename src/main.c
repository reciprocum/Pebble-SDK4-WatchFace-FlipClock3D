/*
   Project: FlipClock3D (watchface)
   File   : main.c
   Author : Afonso Santos, Portugal

   Last revision: 09h45 August 22 2016
*/

#include <pebble.h>
#include <fastmath/FastMath.h>
#include <r3/R3.h>
#include <interpolator/Interpolator.h>
#include <cam3d/Cam3D.h>
#include <transformr3/TransformR3.h>
#include <sampler/Sampler.h>
#include <clock3d/Clock3D.h>

#include "Config.h"


// UI related
static Window  *window ;
static Layer   *world_layer ;


// User related
#define USER_SECONDSINACTIVE_MAX  5

// World related
#define ACCEL_SAMPLER_CAPACITY    8

typedef enum { WORLD_MODE_UNDEFINED
             , WORLD_MODE_LAUNCH
             , WORLD_MODE_DYNAMIC
             , WORLD_MODE_PARK
             , WORLD_MODE_STEADY
             }
WorldMode ;

//OPTION #define TRANSPARENCY_DEFAULT      MESH3D_TRANSPARENCY_XRAY
//OPTION #define TRANSPARENCY_DEFAULT      MESH3D_TRANSPARENCY_WIREFRAME
#define TRANSPARENCY_DEFAULT      MESH3D_TRANSPARENCY_SOLID

// Animation related
#define ANIMATION_INTERVAL_MS     40
#define ANIMATION_FLIP_STEPS      50
#define ANIMATION_SPIN_STEPS      75

static int        world_draw_count    = 0 ;
static WorldMode  world_mode          = WORLD_MODE_UNDEFINED ;
static AppTimer  *world_updateTimer   = NULL ;

Sampler   *sampler_accelX = NULL ;            // To be allocated at world_initialize( ).
Sampler   *sampler_accelY = NULL ;            // To be allocated at world_initialize( ).
Sampler   *sampler_accelZ = NULL ;            // To be allocated at world_initialize( ).

float     *spinRotationFraction    = NULL ;   // To be allocated at world_initialize( ).
float     *animRotationFraction    = NULL ;   // To be allocated at world_initialize( ).
float     *animTranslationFraction = NULL ;   // To be allocated at world_initialize( ).

// User related
static int  user_secondsInactive  = 0 ;


// Spin(Z) CONSTANTS & variables
#define   SPIN_ROTATION_QUANTA    0.0001
#define   SPIN_ROTATION_STEADY   -DEG_045
#define   SPIN_SPEED_AFTER_TWIST  400

static int     spin_speed     = 0 ;                      // Initial spin speed.
static float   spin_rotation  = SPIN_ROTATION_STEADY ;   // Initial spin rotation angle allows to view hours/minutes/seconds faces.


// Camera related
#define  CAM3D_DISTANCEFROMORIGIN   (2.2 * CUBE_SIZE)
#define  CAM3D_VIEWPOINT_STEADY     (R3){ .x = -0.1, .y = 1.0, .z = 0.7 }

static Cam3D    cam ;
static float    cam_zoom = PBL_IF_RECT_ELSE(1.25, 1.14) ;


// Forward declarations.
void  set_world_mode( const WorldMode pWorldMode ) ;
void  world_update_timer_handler( void *data ) ;
void  clock_updateTime( ) ;


// Acellerometer handlers.
void
accel_data_service_handler
( AccelData *data
, uint32_t   num_samples
)
{ }


// Animation related
static int    park_animStep    = -1 ;
static float  park_animRange ;
static int    launch_animStep  = -1 ;
static float  launch_animRange = DEG_090 ;


void
accel_tap_service_handler
( AccelAxisType  axis        // Process tap on ACCEL_AXIS_X, ACCEL_AXIS_Y or ACCEL_AXIS_Z
, int32_t        direction   // Direction is 1 or -1
)
{
  user_secondsInactive = 0 ;      // Tap event qualifies as active user interaction.

  switch (world_mode)
  {
    case WORLD_MODE_LAUNCH:
    case WORLD_MODE_PARK:
    case WORLD_MODE_DYNAMIC:
      switch (axis)
      {
        case ACCEL_AXIS_X:    // Punch: animate everything.
          Clock3D_animateAll( ) ;
          break ;

        case ACCEL_AXIS_Y:    // Twist: spin.
          spin_speed = SPIN_SPEED_AFTER_TWIST ;
          break ;

        case ACCEL_AXIS_Z:
        default:
          break ;
      }

      break ;

    case WORLD_MODE_STEADY:
      switch (axis)
      {
        case ACCEL_AXIS_X:    // Punch: change the display type.
          Clock3D_cycleDisplayType( ) ;
          clock_updateTime( ) ;
          break ;

        case ACCEL_AXIS_Y:    // Twist: change to LAUNCH mode.
          set_world_mode( WORLD_MODE_LAUNCH ) ;
          break ;

        case ACCEL_AXIS_Z:
        default:  // Unknown axis.
          break ;
      }

      break ;

    default:
      break ;
  }
}


static
void
tick_timer_service_handler
( struct tm *tick_time
, TimeUnits  units_changed
)
{
  Clock3D_setTime_DDHHMMSS( tick_time->tm_mday   // days
                          , tick_time->tm_hour   // hours
                          , tick_time->tm_min    // minutes
                          , tick_time->tm_sec    // seconds
                          ) ;

  if (world_mode == WORLD_MODE_DYNAMIC)
  {
    if (spin_speed == 0)
      ++user_secondsInactive ;
  
    // Auto-exit DYNAMIC mode on lack of user interaction.
    if (user_secondsInactive > USER_SECONDSINACTIVE_MAX)
      set_world_mode( WORLD_MODE_PARK ) ;
  }

  // Trigger call to world update. Will launch dynamic mode if needed.
  if (world_updateTimer != NULL)
    app_timer_reschedule( world_updateTimer, 0 ) ;                           // Reschedule next world update.
  else
    world_updateTimer = app_timer_register( 0, world_update_timer_handler, NULL ) ;   // Schedule next world update.
}


void
clock_updateTime
( )
{
  time_t now ;
  time( &now ) ;
  tick_timer_service_handler( localtime( &now ), 0 ) ;        // To set clock digits & dials with current time.
}


void
cam_config
( R3         *viewPoint
, const float rotationZ
)
{
  // setup 3D camera
  Cam3D_lookAtOriginUpwards( &cam
                           , TransformR3_rotateZ( R3_scale( CAM3D_DISTANCEFROMORIGIN    // View point.
                                                          , viewPoint
                                                          )
                                                , rotationZ
                                                )
                           , cam_zoom                                       // Zoom
                           , CAM3D_PROJECTION_PERSPECTIVE
                           ) ;
}


void
set_world_mode
( const WorldMode pWorldMode )
{
  LOGI( "set_world_mode:: pWorldMode = %d", pWorldMode ) ;

  if (pWorldMode == world_mode)
    return ;

  // Start-up entering mode. Subscribe to newly needed services. Apply relevant configurations.
  switch (world_mode = pWorldMode)
  {
    case WORLD_MODE_LAUNCH:
      launch_animStep = ANIMATION_SPIN_STEPS ;

      // Gravity aware.
     	accel_data_service_subscribe( 0, accel_data_service_handler ) ;
    
      // Activate on-second-change clock updates.
      tick_timer_service_subscribe( SECOND_UNIT, tick_timer_service_handler ) ;

      clock_updateTime( ) ;
      break ;

    case WORLD_MODE_DYNAMIC:
      user_secondsInactive = 0 ;   // Reset user inactivity counter.
      break ;

    case WORLD_MODE_PARK:
      park_animStep  = ANIMATION_SPIN_STEPS ;
      park_animRange = spin_rotation - SPIN_ROTATION_STEADY ;    // From current rotation.
      break ;

    case WORLD_MODE_STEADY:
      // Stop previous SECOND_UNIT clock refresh rate.
      tick_timer_service_unsubscribe( ) ;

      // Gravity unaware.
      accel_data_service_unsubscribe( ) ;

      // Next frame will be from the STEADY viewpoint.
      cam_config( &CAM3D_VIEWPOINT_STEADY                  // Unrotated ViewPoint
                , spin_rotation = SPIN_ROTATION_STEADY   // ViewPoint rotation around Z axis.
                ) ;

      // Activate on-minute-change clock updates.
      tick_timer_service_subscribe( MINUTE_UNIT, tick_timer_service_handler ) ;
      break ;

    default:
      break ;
  }
}


static
void
interpolations_initialize
( )
{
  Interpolator_AccelerateDecelerate( spinRotationFraction = malloc((ANIMATION_SPIN_STEPS+1)*sizeof(float))
                                   , ANIMATION_SPIN_STEPS
                                   ) ;

  Interpolator_AccelerateDecelerate( animRotationFraction = malloc((ANIMATION_FLIP_STEPS+1)*sizeof(float))
                                   , ANIMATION_FLIP_STEPS
                                   ) ;

  Interpolator_TrigonometricYoYo( animTranslationFraction = malloc((ANIMATION_FLIP_STEPS+1)*sizeof(float))
                                , ANIMATION_FLIP_STEPS
                                ) ;
}


static
void
sampler_initialize
( )
{
  sampler_accelX = Sampler_new( ACCEL_SAMPLER_CAPACITY ) ;
  sampler_accelY = Sampler_new( ACCEL_SAMPLER_CAPACITY ) ;
  sampler_accelZ = Sampler_new( ACCEL_SAMPLER_CAPACITY ) ;

  for ( int i = 0  ;  i < ACCEL_SAMPLER_CAPACITY  ;  ++i )
  {
    Sampler_push( sampler_accelX,  -81 ) ;   // STEADY viewPoint attractor.
    Sampler_push( sampler_accelY, -816 ) ;   // STEADY viewPoint attractor.
    Sampler_push( sampler_accelZ, -571 ) ;   // STEADY viewPoint attractor.
  }
}


void
world_initialize
( )
{
  Clock3D_initialize( ) ;
  sampler_initialize( ) ;
  interpolations_initialize( ) ;
  Clock3D_config( DIGIT2D_CURVYSKIN ) ;
}


static
void
interpolations_finalize
( )
{
  free( animRotationFraction    ) ; animRotationFraction    = NULL ;
  free( animTranslationFraction ) ; animTranslationFraction = NULL ;
  free( spinRotationFraction    ) ; spinRotationFraction    = NULL ;
}


static
void
sampler_finalize
( )
{
  free( Sampler_free( sampler_accelX ) ) ; sampler_accelX = NULL ;
  free( Sampler_free( sampler_accelY ) ) ; sampler_accelY = NULL ;
  free( Sampler_free( sampler_accelZ ) ) ; sampler_accelZ = NULL ;
}


void
world_finalize
( )
{
  Clock3D_finalize( ) ;
  sampler_finalize( ) ;
  interpolations_finalize( ) ;
}


// UPDATE CAMERA & WORLD OBJECTS PROPERTIES

static
void
world_update
( )
{
  Clock3D_updateAnimation( ANIMATION_FLIP_STEPS ) ;

  if (world_mode != WORLD_MODE_STEADY)
  {
    Clock3D_second100ths_update( ) ;

    AccelData ad ;

    if ( accel_service_peek( &ad ) < 0          // Accel service not available.
      || (world_mode == WORLD_MODE_PARK  &&  park_animStep < ACCEL_SAMPLER_CAPACITY)  // last frames of PARK.
       )
    {
      Sampler_push( sampler_accelX,  -81 ) ;   // STEADY viewPoint attractor.
      Sampler_push( sampler_accelY, -816 ) ;   // STEADY viewPoint attractor.
      Sampler_push( sampler_accelZ, -571 ) ;   // STEADY viewPoint attractor.
    }
    else
    {
#ifdef QEMU
      if (ad.x == 0  &&  ad.y == 0  &&  ad.z == -1000)   // Under QEMU with SENSORS off this is the default output.
      {
        Sampler_push( sampler_accelX,  -81 ) ;
        Sampler_push( sampler_accelY, -816 ) ;
        Sampler_push( sampler_accelZ, -571 ) ;
      }
      else
      {
        Sampler_push( sampler_accelX, ad.x ) ;
        Sampler_push( sampler_accelY, ad.y ) ;
        Sampler_push( sampler_accelZ, ad.z ) ;
      }
#else
      Sampler_push( sampler_accelX, ad.x ) ;
      Sampler_push( sampler_accelY, ad.y ) ;
      Sampler_push( sampler_accelZ, ad.z ) ;
#endif
    }

    // Adjust cam rotation.
    float cam_rotation ;

    switch (world_mode)
    {
      case WORLD_MODE_LAUNCH:
        if (launch_animStep >= 0)
          cam_rotation = SPIN_ROTATION_STEADY  +  (1.0 - spinRotationFraction[launch_animStep--]) * launch_animRange ;
        else
        {
          cam_rotation = spin_rotation = SPIN_ROTATION_STEADY + launch_animRange ;
          set_world_mode( WORLD_MODE_DYNAMIC ) ;
        }

        break ;
  
      case WORLD_MODE_DYNAMIC:
        // Friction: gradualy decrease spin speed until it stops.
        if (spin_speed > 0)
          --spin_speed ;

        if (spin_speed < 0)
          ++spin_speed ;

        if (spin_speed != 0)
          spin_rotation = FastMath_normalizeAngle( spin_rotation + (float)spin_speed * SPIN_ROTATION_QUANTA ) ;

        cam_rotation = spin_rotation ;
        break ;

      case WORLD_MODE_PARK:
        if (park_animStep >= 0)
          cam_rotation = SPIN_ROTATION_STEADY  +  spinRotationFraction[park_animStep--] * park_animRange ;
        else
        {
          cam_rotation = SPIN_ROTATION_STEADY ;
          set_world_mode( WORLD_MODE_STEADY ) ;
        }

        break ;
  
      default:
        cam_rotation = SPIN_ROTATION_STEADY ;
        break ;
    }

    if (world_mode != WORLD_MODE_STEADY)
      cam_config( &(R3){ .x = (float)(sampler_accelX->samplesAcum / sampler_accelX->samplesNum)
                       , .y =-(float)(sampler_accelY->samplesAcum / sampler_accelY->samplesNum)
                       , .z =-(float)(sampler_accelZ->samplesAcum / sampler_accelZ->samplesNum)
                       }
                , cam_rotation
                ) ;
  }

  // this will queue a defered call to the world_draw( ) method.
  layer_mark_dirty( world_layer ) ;
}


void
world_update_timer_handler
( void *data )
{
  world_updateTimer = NULL ;
  world_update( ) ;

  // Call me again ?
  if (world_mode != WORLD_MODE_STEADY  ||  Clock3D_isAnimated( ))
    // Schedule next world_update (next animation frame).
    world_updateTimer = app_timer_register( ANIMATION_INTERVAL_MS
                                          , world_update_timer_handler
                                          , data
                                          ) ;
}


void
world_draw
( Layer    *me
, GContext *gCtx
)
{
  LOGD( "world_draw:: count = %d", ++world_draw_count ) ;

  // Disable antialiasing if running under QEMU (crashes after a few frames otherwise).
#ifdef QEMU
    graphics_context_set_antialiased( gCtx, false ) ;
#endif

  const GRect layerBounds = layer_get_bounds( me ) ;
  const uint8_t w = layerBounds.size.w ;
  const uint8_t h = layerBounds.size.h ;
 
  Clock3D_draw( gCtx, &cam, w, h, TRANSPARENCY_DEFAULT ) ;
}


void
window_load
( Window *window )
{
  Layer *window_root_layer = window_get_root_layer( window ) ;

  GRect bounds = layer_get_frame( window_root_layer ) ;
  world_layer = layer_create( bounds ) ;
  layer_set_update_proc( world_layer, world_draw ) ;
  layer_add_child( window_root_layer, world_layer ) ;

  // Become tap aware.
  accel_tap_service_subscribe( accel_tap_service_handler ) ;

  // Set initial world mode.
  set_world_mode( WORLD_MODE_LAUNCH ) ;
}


void
window_unload
( Window *window )
{
  // Stop world animation.
  if (world_updateTimer != NULL)
  {
    app_timer_cancel( world_updateTimer ) ;
    world_updateTimer = NULL ;
  }

  // Stop clock.
  tick_timer_service_unsubscribe( ) ;

  // Gravity unaware.
  accel_data_service_unsubscribe( ) ;

  // Tap unaware.
  accel_tap_service_unsubscribe( ) ;

  layer_destroy( world_layer ) ;
}


void
app_initialize
( void )
{
  world_initialize( ) ;

  window = window_create( ) ;
  window_set_background_color( window, GColorBlack ) ;
 
  window_set_window_handlers( window
                            , (WindowHandlers)
                              { .load   = window_load
                              , .unload = window_unload
                              }
                            ) ;

  window_stack_push( window, false ) ;
}


void
app_finalize
( void )
{
  window_stack_remove( window, false ) ;
  window_destroy( window ) ;
  world_finalize( ) ;
}


int
main
( void )
{
  app_initialize( ) ;
  app_event_loop( ) ;
  app_finalize( ) ;
}