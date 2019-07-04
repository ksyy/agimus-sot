#include <queue>

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <tf/transform_broadcaster.h>

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/factory.h>

#include <sot/core/matrix-geometry.hh>

#include <dynamic_graph_bridge/ros_init.hh>


namespace dynamicgraph {
  class RosTfBroadcaster;

  namespace internal
  {
    typedef std::pair<tf::StampedTransform, int> Tf_time_t;
    typedef std::queue<Tf_time_t> Queue_t;


    struct TransformBroadcasterData {
      typedef Signal<sot::MatrixHomogeneous, int> signal_t;

      Queue_t& broadcaster_queue;
      const std::string toFrame, fromFrame;
      tf::StampedTransform transform;
      signal_t signal;

      TransformBroadcasterData (Queue_t& b,
          const std::string& to, const std::string& from,
          const std::string& signame)
        : broadcaster_queue (b)
        , toFrame (to)
        , fromFrame (from)
        , signal (signame)
      {
        transform.frame_id_ = fromFrame;
        transform.child_frame_id_ = toFrame;
        signal.setFunction (boost::bind(&TransformBroadcasterData::sendTransform, this, _1, _2));
      }

      sot::MatrixHomogeneous& sendTransform (sot::MatrixHomogeneous& data, int time)
      {
        transform.stamp_ = ros::Time::now();
        for (sot::MatrixHomogeneous::Index r = 0; r < 3; ++r) {
          for (sot::MatrixHomogeneous::Index c = 0; c < 3; ++c)
            transform.getBasis()[r][c] = data.linear ()(r,c);
          transform.getOrigin()[r] = data.translation()[r];
        }
        
        // We do not care about thread-safety because the order of the pushes is not important
        // and only one thread is poping
        broadcaster_queue.push(Tf_time_t(transform, time));
        return data;
      }
    };

    struct TransformBroadcaster_thread
    {
      bool requestShutdown_;
      boost::thread t_;

      TransformBroadcaster_thread (RosTfBroadcaster* ros_tf_broadcaster);

      void spin (RosTfBroadcaster* ros_tf_broadcaster);
    };
  } // end of internal namespace.

  class RosTfBroadcaster : public Entity
  {
    DYNAMIC_GRAPH_ENTITY_DECL();

    public:
      typedef internal::TransformBroadcasterData TransformBroadcasterData; 

      RosTfBroadcaster (const std::string& name);

      ~RosTfBroadcaster ();

      void add (const std::string& to, const std::string& from, const std::string& signame);

      void broadcast();
 
    private:
      typedef std::map<std::string, TransformBroadcasterData*> Map_t;
      Map_t broadcasterData;
      tf::TransformBroadcaster broadcaster;
      internal::Queue_t tf_queue;
      internal::TransformBroadcaster_thread *thread_;
  };
  
} // end of namespace dynamicgraph.

