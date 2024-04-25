#include <libcamera_ros/utils/type_extent.h>
#include <libcamera/base/span.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <stdexcept>
#include <string>
#include <type_traits>


template<typename T, std::enable_if_t<!libcamera::details::is_span<T>::value, bool> = true>
std::size_t
get_extent(const libcamera::Control<T> &)
{
  return 0;
}

template<typename T, std::enable_if_t<libcamera::details::is_span<T>::value, bool> = true>
std::size_t
get_extent(const libcamera::Control<T> &)
{
  return libcamera::Control<T>::type::extent;
}

#define IF(T)                                                                                      \
  if (id->id() == libcamera::controls::T.id())                                                     \
    return get_extent(libcamera::controls::T);


std::size_t
get_extent(const libcamera::ControlId *id)
{
  IF(AeEnable)
  IF(AeLocked)
  IF(AeMeteringMode)
  IF(AeConstraintMode)
  IF(AeExposureMode)
  IF(ExposureValue)
  IF(ExposureTime)
  IF(AnalogueGain)
  /* IF(AeFlickerMode) */
  /* IF(AeFlickerPeriod) */
  /* IF(AeFlickerDetected) */
  IF(Brightness)
  IF(Contrast)
  IF(Lux)
  IF(AwbEnable)
  IF(AwbMode)
  IF(AwbLocked)
  IF(ColourGains)
  IF(ColourTemperature)
  IF(Saturation)
  IF(SensorBlackLevels)
  IF(Sharpness)
  IF(FocusFoM)
  IF(ColourCorrectionMatrix)
  IF(ScalerCrop)
  IF(DigitalGain)
  IF(FrameDuration)
  IF(FrameDurationLimits)
  /* IF(SensorTemperature) */
  IF(SensorTimestamp)
  IF(AfMode)
  IF(AfRange)
  IF(AfSpeed)
  IF(AfMetering)
  IF(AfWindows)
  IF(AfTrigger)
  IF(AfPause)
  IF(LensPosition)
  IF(AfState)
  IF(AfPauseState)
  /* IF(HdrMode) */
  /* IF(HdrChannel) */

  throw std::runtime_error("control " + id->name() + " (" + std::to_string(id->id()) +
                           ") not handled");
}
