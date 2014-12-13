#ifndef FLOOR_POINTER_H
#define FLOOR_POINTER_H

#include <vector>
#include <NiTE.h>
#include <cmath>
#include <ros/ros.h>

class FloorPointer
{
public:
  float straightness;
  double minHipDistance;
  std::vector<nite::Point3f> leftHandPositionHistory;
  std::vector<nite::Point3f> rightHandPositionHistory;
  float average[3];
  enum Side {NONE=1, LEFT, RIGHT};
  int maxQueuesize;
  float accuracyTreshold;

  FloorPointer();

/////////////////////////////////////////////////////////////////////
  /**
   * Calculates the Euclidean distance between two points in 3D.
   * @param firstPoint 3D point
   * @param secondPoint 3D point
   * @return Distance in meters.
   */
  double getDistance(const nite::Point3f& firstPoint, const nite::Point3f& secondPoint) const;
/////////////////////////////////////////////////////////////////////
  double getLeftArmLength(const nite::Point3f& leftShoulderPoint, const nite::Point3f& leftElbowPoint,
                          const nite::Point3f& leftHandPoint);
/////////////////////////////////////////////////////////////////////
  double getRightArmLength(const nite::Point3f& rightShoulderPoint, const nite::Point3f& rightElbowPoint,
                           const nite::Point3f& rightHandPoint);
/////////////////////////////////////////////////////////////////////
  float calcDotProduct(nite::Point3f& point1, nite::Point3f& point2);
/////////////////////////////////////////////////////////////////////
  nite::Point3f subPoints(nite::Point3f& point1, nite::Point3f& point2);
/////////////////////////////////////////////////////////////////////
  nite::Point3f addPoints(nite::Point3f& point1, nite::Point3f& point2);
/////////////////////////////////////////////////////////////////////
  Side getReliableGestureSide(nite::Point3f& leftHand, const nite::Point3f& rightHand, const Side& side);
/////////////////////////////////////////////////////////////////////
  bool isAverage(const Side& side);
/////////////////////////////////////////////////////////////////////
  void updateHistory(const Side& side, const nite::Point3f& point = nite::Point3f(0, 0, 0));
/////////////////////////////////////////////////////////////////////
  Side getPointingSide(const nite::Point3f& headPoint, const nite::Point3f& leftShoulderPoint,
                      const nite::Point3f& leftElbowPoint, const nite::Point3f& leftHandPoint,
                      const nite::Point3f& leftHipPoint, const nite::Point3f& rightShoulderPoint,
                      const nite::Point3f& rightElbowPoint, const nite::Point3f& rightHandPoint,
                      const nite::Point3f& rightHipPoint);
/////////////////////////////////////////////////////////////////////////////////////
  bool isFloorPoint(const nite::Skeleton& userSkeleton, const nite::Plane& floor, nite::Point3f& Output);

  friend class FloorPointerTest_testIsGestureReliable_Test;
};
#endif
