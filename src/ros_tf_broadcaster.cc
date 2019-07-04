# include <boost/bind.hpp>

# include <tf/transform_broadcaster.h>

# include <dynamic-graph/entity.h>
# include <dynamic-graph/signal.h>
# include <dynamic-graph/command-bind.h>
# include <dynamic-graph/factory.h>

# include <sot/core/matrix-geometry.hh>

# include <dynamic_graph_bridge/ros_init.hh>

namespace dynamicgraph {
  class RosTfBroadcaster;

  namespace internal
  {
    struct TransformBroadcasterData {
      typedef Signal<sot::MatrixHomogeneous, int> signal_t;

      tf::TransformBroadcaster& broadcaster;
      const std::string toFrame, fromFrame;
      tf::StampedTransform transform;
      signal_t signal;

      TransformBroadcasterData (tf::TransformBroadcaster& b,
          const std::string& to, const std::string& from,
          const std::string& signame)
        : broadcaster (b)
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
        try {
          broadcaster.sendTransform (transform);
        } catch (const tf::TransformException& ex) {
          ROS_ERROR("Enable to send transform at time %i: %s",time,ex.what());
          return data;
        }
        return data;
      }
    };
  } // end of internal namespace.

  class RosTfBroadcaster : public Entity
  {
    DYNAMIC_GRAPH_ENTITY_DECL();

    public:
      typedef internal::TransformBroadcasterData TransformBroadcasterData; 

      RosTfBroadcaster (const std::string& name) : Entity (name)
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

      ~RosTfBroadcaster ()
      {
        for (Map_t::const_iterator _it = broadcasterData.begin(); _it != broadcasterData.end(); ++_it)
          delete _it->second;
      }

      void add (const std::string& to, const std::string& from, const std::string& signame)
      {
        if (broadcasterData.find(signame) != broadcasterData.end())
          throw std::invalid_argument ("A signal " + signame
              + " already exists in RosTfBroadcaster " + getName());

        boost::format signalName ("RosTfBroadcaster(%1%)::input(MatrixHomo)::%2%");
        signalName % getName () % signame;

        TransformBroadcasterData* tld = new TransformBroadcasterData (
            broadcaster, to, from, signalName.str());
        signalRegistration (tld->signal);
        broadcasterData[signame] = tld;
      }

    private:
      typedef std::map<std::string, TransformBroadcasterData*> Map_t;
      Map_t broadcasterData;
      tf::TransformBroadcaster broadcaster;
  };
  
  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosTfBroadcaster, "RosTfBroadcaster");
} // end of namespace dynamicgraph.

