#include "ros_tf_broadcaster.hh"


namespace dynamicgraph
{
    namespace internal
    {
        TransformBroadcaster_thread::TransformBroadcaster_thread (RosTfBroadcaster* ros_tf_broadcaster)
        : requestShutdown_ (false)
        , t_ (&TransformBroadcaster_thread::spin, this, ros_tf_broadcaster)
      {}

      void TransformBroadcaster_thread::spin (RosTfBroadcaster* ros_tf_broadcaster)
      {
        // Change the thread's scheduler from real-time to normal and reduce its priority
        int threadPolicy;
        struct sched_param threadParam;
        if (pthread_getschedparam (pthread_self(), &threadPolicy, &threadParam) == 0)
        {
          threadPolicy = SCHED_OTHER;
          threadParam.sched_priority -= 5;
          if (threadParam.sched_priority < sched_get_priority_min (threadPolicy))
            threadParam.sched_priority = sched_get_priority_min (threadPolicy);

          pthread_setschedparam (pthread_self(), threadPolicy, &threadParam);
        }

        while (!requestShutdown_)
        {
          ros_tf_broadcaster->broadcast();
          boost::this_thread::sleep(boost::posix_time::milliseconds(10));
        }
      }
    }
      
    RosTfBroadcaster::RosTfBroadcaster (const std::string& name)
        : Entity (name)
        , thread_(new internal::TransformBroadcaster_thread(this))
      {
        std::string docstring =
          "\n"
          "  Add a signal containing the transform between two frames.\n"
          "\n"
          "  Input:\n"
          "    - to  : frame name\n"
          "    - from: frame name,\n"
          "    - signalName: the signal name in dynamic-graph"
          "\n";
        addCommand ("add",
            command::makeCommandVoid3(*this, &RosTfBroadcaster::add, docstring));
      }

    RosTfBroadcaster::~RosTfBroadcaster ()
      {
        thread_->requestShutdown_ = true;
        thread_->t_.join();
        delete thread_;
        for (Map_t::const_iterator _it = broadcasterData.begin(); _it != broadcasterData.end(); ++_it)
          delete _it->second;
      }

      void RosTfBroadcaster::add (const std::string& to, const std::string& from, const std::string& signame)
      {
        if (broadcasterData.find(signame) != broadcasterData.end())
          throw std::invalid_argument ("A signal " + signame
              + " already exists in RosTfBroadcaster " + getName());

        boost::format signalName ("RosTfBroadcaster(%1%)::input(MatrixHomo)::%2%");
        signalName % getName () % signame;

        TransformBroadcasterData* tld = new TransformBroadcasterData (
            tf_queue, to, from, signalName.str());
        signalRegistration (tld->signal);
        broadcasterData[signame] = tld;
      }

      void RosTfBroadcaster::broadcast()
      {
        // We do not care about thread-safety because the order of the pushes is not important
        // and only this thread is poping
        while(!tf_queue.empty())
        {
          internal::Tf_time_t transform = tf_queue.front();
          try {
            broadcaster.sendTransform (transform.first);
          } catch (const tf::TransformException& ex) {
            ROS_ERROR("Unable to send transform from time %i: %s", transform.second, ex.what());
          }
          tf_queue.pop();
        }
      }

      
  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosTfBroadcaster, "RosTfBroadcaster");
}
