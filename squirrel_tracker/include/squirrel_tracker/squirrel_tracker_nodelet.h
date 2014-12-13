#include <nodelet/nodelet.h>
#include "squirrel_tracker/squirrel_tracker.h"

class SquirrelTrackerNodelet : public nodelet::Nodelet
{
public:
  SquirrelTrackerNodelet();
  virtual ~SquirrelTrackerNodelet();
  virtual void onInit();
private:
  SquirrelTracker *listener;
};

