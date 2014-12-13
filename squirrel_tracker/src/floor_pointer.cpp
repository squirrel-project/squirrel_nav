#include "squirrel_tracker/floor_pointer.h"

FloorPointer::FloorPointer() :
    average {0.0}
{
  straightness = 0.9;
  minHipDistance = 0.3;
  maxQueuesize = 10;
  accuracyTreshold = 0.03;
}

/////////////////////////////////////////////////////////////////////
double FloorPointer::getDistance(const nite::Point3f& firstPoint, const nite::Point3f& secondPoint) const
{
  const double distance = sqrt(
      pow((secondPoint.x - firstPoint.x), 2) + pow((secondPoint.y - firstPoint.y), 2)
          + pow((secondPoint.z - firstPoint.z), 2));
  return (distance / 1000);

}
/////////////////////////////////////////////////////////////////////
double FloorPointer::getLeftArmLength(const nite::Point3f& leftShoulderPoint, const nite::Point3f& leftElbowPoint,
                                      const nite::Point3f& leftHandPoint)
{
  double leftArmLength = getDistance(leftShoulderPoint, leftElbowPoint) + getDistance(leftElbowPoint, leftHandPoint);
  return leftArmLength;
}
/////////////////////////////////////////////////////////////////////
double FloorPointer::getRightArmLength(const nite::Point3f& rightShoulderPoint, const nite::Point3f& rightElbowPoint,
                                       const nite::Point3f& rightHandPoint)
{
  double rightArmLength = getDistance(rightShoulderPoint, rightElbowPoint)
      + getDistance(rightElbowPoint, rightHandPoint);
  return rightArmLength;
}
/////////////////////////////////////////////////////////////////////
float FloorPointer::calcDotProduct(nite::Point3f& point1, nite::Point3f& point2)
{
  return (point1.x * point2.x) + (point1.y * point2.y) + (point1.z * point2.z);
}
/////////////////////////////////////////////////////////////////////
nite::Point3f FloorPointer::subPoints(nite::Point3f& point1, nite::Point3f& point2)
{
  nite::Point3f point;
  point.x = (point1.x - point2.x);
  point.y = (point1.y - point2.y);
  point.z = (point1.z - point2.z);
  return point;
}
/////////////////////////////////////////////////////////////////////
nite::Point3f FloorPointer::addPoints(nite::Point3f& point1, nite::Point3f& point2)
{
  nite::Point3f point;
  point.x = (point1.x + point2.x);
  point.y = (point1.y + point2.y);
  point.z = (point1.z + point2.z);
  return point;
}
/////////////////////////////////////////////////////////////////////
FloorPointer::Side FloorPointer::getReliableGestureSide(nite::Point3f& leftHand, const nite::Point3f& rightHand,
                                                        const Side& side)
{

  if (!isAverage(side))
  {
    return NONE;
  }
  nite::Point3f point1 = {average[0], average[1], average[2]};
  if (side == LEFT)
  {
    if (getDistance(point1, leftHand) < accuracyTreshold)
    {
      return LEFT;
    }
    else
    {
      return NONE;
    }
  }
  if (side == RIGHT)
  {
    if (getDistance(point1, rightHand) < accuracyTreshold)
    {
      return RIGHT;
    }
    else
    {
      return NONE;
    }
  }
  return NONE;
}
/////////////////////////////////////////////////////////////////////
bool FloorPointer::isAverage(const Side& side)
{
  average[0] = 0.0;
  average[1] = 0.0;
  average[2] = 0.0;
  if (side == LEFT)
  {
    int n = leftHandPositionHistory.size();
    if (n < maxQueuesize)
    {
      return false;
    }
    for (int i = 0; i < n; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        if (j == 0)
        {
          average[j] = (average[j] + leftHandPositionHistory[i].x / n);
        }
        else if (j == 1)
        {
          average[j] = (average[j] + leftHandPositionHistory[i].y / n);
        }
        else
        {
          average[j] = (average[j] + leftHandPositionHistory[i].z / n);
        }
      }
    }
  }
  if (side == RIGHT)
  {
    int n = rightHandPositionHistory.size();
    if (n < maxQueuesize)
    {
      return false;
    }
    for (int i = 0; i < n; i++)
    {

      for (int j = 0; j < 3; j++)
      {
        if (j == 0)
        {
          average[j] = (average[j] + rightHandPositionHistory[i].x / n);
        }
        else if (j == 1)
        {
          average[j] = (average[j] + rightHandPositionHistory[i].y / n);
        }
        else
        {
          average[j] = (average[j] + rightHandPositionHistory[i].z / n);
        }
      }
    }
  }
  if (side == NONE)
  {
    return false;
  }
  return true;
}
/////////////////////////////////////////////////////////////////////
void FloorPointer::updateHistory(const Side& side, const nite::Point3f& point)
{
  switch (side)
  {
    case LEFT:
      leftHandPositionHistory.insert(leftHandPositionHistory.begin(), point);
      if (leftHandPositionHistory.size() > maxQueuesize)
        leftHandPositionHistory.pop_back();
      break;
    case RIGHT:
      rightHandPositionHistory.insert(rightHandPositionHistory.begin(), point);
      if (rightHandPositionHistory.size() > maxQueuesize)
        rightHandPositionHistory.pop_back();
      break;
    case NONE:
      rightHandPositionHistory.clear();
      leftHandPositionHistory.clear();
      break;
  }
}
/////////////////////////////////////////////////////////////////////
FloorPointer::Side FloorPointer::getPointingSide(const nite::Point3f& headPoint, const nite::Point3f& leftShoulderPoint,
                                                 const nite::Point3f& leftElbowPoint,
                                                 const nite::Point3f& leftHandPoint, const nite::Point3f& leftHipPoint,
                                                 const nite::Point3f& rightShoulderPoint,
                                                 const nite::Point3f& rightElbowPoint,
                                                 const nite::Point3f& rightHandPoint,
                                                 const nite::Point3f& rightHipPoint)
{

  if (getDistance(leftShoulderPoint, leftHandPoint)
      > straightness * getLeftArmLength(leftShoulderPoint, leftElbowPoint, leftHandPoint)
      && getDistance(leftHandPoint, leftHipPoint) > minHipDistance && (headPoint.y - leftHandPoint.y) > 0)
  {
    updateHistory(LEFT, leftHandPoint);
    return LEFT;
  }
  else if (getDistance(rightShoulderPoint, rightHandPoint)
      > straightness * getRightArmLength(rightShoulderPoint, rightElbowPoint, rightHandPoint)
      && getDistance(rightHandPoint, rightHipPoint) > minHipDistance && (headPoint.y - rightHandPoint.y) > 0)
  {
    updateHistory(RIGHT, rightHandPoint);
    return RIGHT;
  }
  else
  {
    updateHistory(NONE);
    return NONE;
  }
}
/////////////////////////////////////////////////////////////////////////////////////
bool FloorPointer::isFloorPoint(const nite::Skeleton& userSkeleton, const nite::Plane& floor, nite::Point3f& Output)
{
  nite::Point3f headPoint = userSkeleton.getJoint(nite::JOINT_HEAD).getPosition();
  nite::Point3f rightShoulderPoint = userSkeleton.getJoint(nite::JOINT_RIGHT_SHOULDER).getPosition();

  nite::Point3f rightHandPoint = userSkeleton.getJoint(nite::JOINT_RIGHT_HAND).getPosition();

  nite::Point3f leftShoulderPoint = userSkeleton.getJoint(nite::JOINT_LEFT_SHOULDER).getPosition();

  nite::Point3f leftHandPoint = userSkeleton.getJoint(nite::JOINT_LEFT_HAND).getPosition();

  nite::Point3f leftHipPoint = userSkeleton.getJoint(nite::JOINT_LEFT_HIP).getPosition();

  nite::Point3f rightHipPoint = userSkeleton.getJoint(nite::JOINT_RIGHT_HIP).getPosition();

  nite::Point3f rightElbowPoint = userSkeleton.getJoint(nite::JOINT_RIGHT_ELBOW).getPosition();

  nite::Point3f leftElbowPoint = userSkeleton.getJoint(nite::JOINT_LEFT_ELBOW).getPosition();

  if (getReliableGestureSide(
      leftHandPoint,
      rightHandPoint,
      getPointingSide(headPoint, leftShoulderPoint, leftElbowPoint, leftHandPoint, leftHipPoint, rightShoulderPoint,
                      rightElbowPoint, rightHandPoint, rightHipPoint)) == LEFT)
  {
    nite::Point3f FloorNormal = {floor.normal.x, floor.normal.y, floor.normal.z};
    nite::Point3f FloorPoint = {floor.point.x, floor.point.y, floor.point.z};
//      nite::Point3f vecShoulderHandleft = subPoints(leftShoulderPoint, leftHandPoint);
    nite::Point3f vecHeadHandleft = subPoints(headPoint, leftHandPoint);
    nite::Point3f vecHandFloorPoint = subPoints(FloorPoint, leftHandPoint);
    float normFaktor = calcDotProduct(FloorNormal, vecHandFloorPoint) / calcDotProduct(FloorNormal, vecHeadHandleft);
    nite::Point3f normvecSholderHand = {vecHeadHandleft.x * normFaktor, vecHeadHandleft.y * normFaktor,
                                        vecHeadHandleft.z * normFaktor};
    nite::Point3f intersectionPoint = addPoints(leftHandPoint, normvecSholderHand);
    Output = intersectionPoint;
    return true;
  }
  else if (getReliableGestureSide(
      leftHandPoint,
      rightHandPoint,
      getPointingSide(headPoint, leftShoulderPoint, leftElbowPoint, leftHandPoint, leftHipPoint, rightShoulderPoint,
                      rightElbowPoint, rightHandPoint, rightHipPoint)) == RIGHT)
  {

    nite::Point3f FloorNormal = {floor.normal.x, floor.normal.y, floor.normal.z};
    nite::Point3f FloorPoint = {floor.point.x, floor.point.y, floor.point.z};
//      nite::Point3f vecShoulderHandright = subPoints(rightShoulderPoint, rightHandPoint);
    nite::Point3f vecHeadHandright = subPoints(headPoint, rightHandPoint);
    nite::Point3f vecHandFloorPoint = subPoints(FloorPoint, rightHandPoint);
    float normFaktor = calcDotProduct(FloorNormal, vecHandFloorPoint) / calcDotProduct(FloorNormal, vecHeadHandright);
    nite::Point3f normvecSholderHand = {vecHeadHandright.x * normFaktor, vecHeadHandright.y * normFaktor,
                                        vecHeadHandright.z * normFaktor};
    nite::Point3f intersectionPoint = addPoints(rightHandPoint, normvecSholderHand);
    Output = intersectionPoint;
    return true;
  }
  else
  {
    return false;
  }
}

