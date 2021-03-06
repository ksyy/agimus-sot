from __future__ import print_function
from tools import Manifold, Posture
from dynamic_graph import plug

def _hpTasks (sotrobot):
    return Manifold()
def _lpTasks (sotrobot):
    return Posture ("posture", sotrobot)

## Supervise the consecutive execution of several SoT.
#
# Typically, these sots are created via factory.Factory. They can also be added manually.
class Supervisor(object):
    """
    Steps: P = placement, G = grasp, p = pre-P, g = pre-G
    0. P <-> GP
    1. P <-> gP
    2. gP <-> GP
    3. GP <-> G
    4. GP <-> Gp
    5. Gp <-> G
    """
    ## \param lpTasks function taking as input "sotrobot" and return low priority task
    #         \todo this should not be a function but a set of tasks.
    ## \param hpTasks function taking as input "sotrobot" and return high priority task (like balance)
    #         \todo this should not be a function but a set of tasks.
    def __init__ (self, sotrobot, lpTasks = None, hpTasks = None):
        self.sotrobot = sotrobot
        self.hpTasks = hpTasks if hpTasks is not None else _hpTasks(sotrobot)
        self.lpTasks = lpTasks if lpTasks is not None else _lpTasks(sotrobot)
        self.currentSot = None
        from dynamic_graph.sot.core.switch import SwitchVector
        self.sot_switch = SwitchVector ("sot_supervisor_switch")
        plug(self.sot_switch.sout, self.sotrobot.device.control)

        from agimus_sot.events import Events
        self. done_events = Events ("done" , sotrobot)
        self.error_events = Events ("error", sotrobot)
        self. done_events.setupNormOfControl (sotrobot.device.control, 1e-2)
        self. done_events.setupTime () # For signal self. done_events.timeEllapsedSignal
        self.error_events.setupTime () # For signal self.error_events.timeEllapsedSignal

    def makeInitialSot (self):
        # Create the initial sot (keep)
        from .solver import Solver
        sot = Solver ('sot_keep', self.sotrobot.dynamic.getDimension())

        self.keep_posture = Posture ("posture_keep", self.sotrobot)
        self.keep_posture.tp.setWithDerivative (False)
        self.keep_posture._signalPositionRef().value = self.sotrobot.dynamic.position.value
        
        self.keep_posture.pushTo(sot)
        sot. doneSignal = self.controlNormConditionSignal()
        sot.errorSignal = False
        self.addSolver ("", sot)

    ## Set the robot base pose in the world.
    # \param basePose a list: [x,y,z,r,p,y] or [x,y,z,qx,qy,qz,qw]
    # \return success True in case of success
    def setBasePose (self, basePose):
        if len(basePose) == 7:
            # Currently, this case never happens
            from dynamic_graph.sot.tools.quaternion import Quaternion
            from numpy.linalg import norm
            q = Quaternion(basePose[6],basePose[3],basePose[4],basePose[5])
            if abs(norm(q.array) - 1.) > 1e-2:
              return False, "Quaternion is not normalized"
            basePose = basePose[:3] + q.toRPY().tolist()
        if self.currentSot == "" or len(basePose) != 6:
            # We are using the SOT to keep the current posture.
            # The 6 first DoF are not used by the task so we can change them safely.
            self.sotrobot.device.set(tuple(basePose + list(self.sotrobot.device.state.value[6:])))
            self.keep_posture._signalPositionRef().value = self.sotrobot.device.state.value
            return True
        else:
            return False

    ## \name SoT managements
    ##  \{

    def addPreAction (self, name, preActionSolver):
        self.preActions[name] = preActionSolver
        self._addSignalToSotSwitch (preActionSolver)

    def addSolver (self, name, solver):
        self.sots[name] = solver
        self._addSignalToSotSwitch (solver)

    def duplicateSolver (self, existingSolver, newSolver):
        self.sots[newSolver] = self.sots[existingSolver]

    def addPostActions (self, name, postActionSolvers):
        self.postActions[name] = postActionSolvers
        for targetState, pa_sot in postActionSolvers.iteritems():
            self._addSignalToSotSwitch (pa_sot)

    ## This is for internal purpose
    def _addSignalToSotSwitch (self, solver):
        n = self.sot_switch.getSignalNumber()
        self.sot_switch.setSignalNumber(n+1)
        self.sots_indexes[solver.name] = n
        plug (solver.control, self.sot_switch.signal("sin" + str(n)))

        def _plug (e, events, n, name):
            assert events.getSignalNumber() == n, "Wrong number of events."
            events.setSignalNumber(n+1)
            events.setConditionString(n, name)
            if isinstance(e, (bool,int)): events.conditionSignal(n).value = int(e)
            else: plug (e, events.conditionSignal(n))

        _plug (solver. doneSignal, self. done_events, n, solver.name)
        _plug (solver.errorSignal, self.error_events, n, solver.name)

    def _selectSolver (self, solver):
        n = self.sots_indexes[solver.name]
        self.  sot_switch.selection.value = n
        self. done_events.setSelectedSignal(n)
        self.error_events.setSelectedSignal(n)

    ## \}

    def controlNormConditionSignal (self):
        return self. done_events.controlNormSignal

    def topics (self):
        c = self.hpTasks + self.lpTasks
        for g in self.grasps.values():
            c += g

        return c.topics

    def plugTopicsToRos (self):
        from dynamic_graph.ros.ros_queued_subscribe import RosQueuedSubscribe
        self.rosSubscribe = RosQueuedSubscribe ('ros_queued_subscribe')
        from dynamic_graph.ros.ros_tf_listener import RosTfListener
        self.rosTf = RosTfListener ('ros_tf_listener')
        topics = self.topics()

        for name, topic_info in topics.items():
            topic_handler = _handlers[topic_info.get("handler","default")]
            topic_handler (name,topic_info,self.rosSubscribe,self.rosTf)

    def printQueueSize (self):
        exec ("tmp = " + self.rosSubscribe.list())
        for l in tmp: print (l, self.rosSubscribe.queueSize(l))

    ## Check consistency between two SoTs.
    #
    # This is not used anymore because it must be synchronized with the real-time thread.
    # \todo Re-enable consistency check between two SoTs.
    def isSotConsistentWithCurrent(self, transitionName, thr = 1e-3):
        if self.currentSot is None or transitionName == self.currentSot:
            return True
        csot = self.sots[self.currentSot]
        nsot = self.sots[transitionName]
        t = self.sotrobot.device.control.time
        # This is not safe since it would be run concurrently with the
        # real time thread.
        csot.control.recompute(t)
        nsot.control.recompute(t)
        from numpy import array, linalg
        error = array(nsot.control.value) - array(csot.control.value)
        n = linalg.norm(error)
        if n > thr:
            print ("Control not consistent:", linalg.norm(error),'\n', error)
            return False
        return True

    def clearQueues(self):
        self.rosSubscribe.readQueue (-1)
        exec ("tmp = " + self.rosSubscribe.list())
        for s in tmp:
            self.rosSubscribe.clearQueue(s)

    ## Start reading values received by the RosQueuedSubscribe entity.
    # \param delay (integer) how many periods to wait before reading.
    #              It allows to give some delay to network connection.
    # \param minQueueSize (integer) waits to the queue size of rosSubscribe
    #                     to be greater or equal to \p minQueueSize
    # \param duration expected duration (in seconds) of the queue.
    #
    # \warning If \p minQueueSize is greater than the number of values to
    #          be received by rosSubscribe, this function does an infinite loop.
    def readQueue(self, delay, minQueueSize, duration):
        from time import sleep
        if delay < 0:
            print ("Delay argument should be >= 0")
            return
        while self.rosSubscribe.queueSize("posture") < minQueueSize:
            sleep(0.001)
        durationStep = int(duration / self.sotrobot.device.getTimeStep())
        t = self.sotrobot.device.control.time + delay
        self.rosSubscribe.readQueue (t)
        self. done_events.setFutureTime (t + durationStep)
        self.error_events.setFutureTime (t + durationStep)

    def stopReadingQueue(self):
        self.rosSubscribe.readQueue (-1)

    def plugSot(self, transitionName, check = False):
        if check and not self.isSotConsistentWithCurrent (transitionName):
            # raise Exception ("Sot %d not consistent with sot %d" % (self.currentSot, id))
            print("Sot {0} not consistent with sot {1}".format(self.currentSot, transitionName))
        if transitionName == "":
            self.keep_posture._signalPositionRef().value = self.sotrobot.dynamic.position.value
        solver = self.sots[transitionName]

        # No done events should be triggered before call
        # to readQueue. We expect it to happen with 1e6 milli-seconds
        # from now...
        devicetime = self.sotrobot.device.control.time
        self. done_events.setFutureTime (devicetime + 100000)

        self._selectSolver (solver)
        print("{0}: Current solver {1}\n{2}"
                .format(devicetime, transitionName, solver.sot.display()))
        self.currentSot = transitionName

    def runPreAction(self, transitionName):
        if self.preActions.has_key(transitionName):
            solver = self.preActions[transitionName]

            t = self.sotrobot.device.control.time + 2
            self. done_events.setFutureTime (t)

            self._selectSolver (solver)
            print("{0}: Running pre action {1}\n{2}"
                    .format(t, transitionName, solver.sot.display()))
            return True
        print ("No pre action", transitionName)
        return False

    def runPostAction(self, targetStateName):
        if self.postActions.has_key(self.currentSot):
            d = self.postActions[self.currentSot]
            if d.has_key(targetStateName):
                solver = d[targetStateName]

                devicetime = self.sotrobot.device.control.time
                self. done_events.setFutureTime (devicetime + 2)

                self._selectSolver (solver)

                print("{0}: Running post action {1} --> {2}\n{3}"
                        .format(devicetime, self.currentSot, targetStateName,
                            solver.sot.display()))
                return True
        print ("No post action {0} --> {1}".format(self.currentSot, targetStateName))
        return False

    def getJointList (self, prefix = ""):
        return [ prefix + n for n in self.sotrobot.dynamic.model.names[1:] ]

    def publishState (self, subsampling = 40):
        if hasattr (self, "ros_publish_state"):
            return
        from dynamic_graph.ros import RosPublish
        self.ros_publish_state = RosPublish ("ros_publish_state")
        self.ros_publish_state.add ("vector", "state", "/agimus/sot/state")
        self.ros_publish_state.add ("vector", "reference_state", "/agimus/sot/reference_state")
        plug (self.sotrobot.device.state, self.ros_publish_state.state)
        plug (self.rosSubscribe.posture, self.ros_publish_state.reference_state)
        self.sotrobot.device.after.addDownsampledSignal ("ros_publish_state.trigger", subsampling)

def _defaultHandler(name,topic_info,rosSubscribe,rosTf):
    topic = topic_info["topic"]
    rosSubscribe.add (topic_info["type"], name, topic)
    for s in topic_info['signalGetters']:
        plug (rosSubscribe.signal(name), s())
    print (topic, "plugged to", name, ', ', len(topic_info['signalGetters']), 'times')

def _handleTfListener (name,topic_info,rosSubscribe,rosTf):
    signame = topic_info["frame1"] + "_wrt_" + topic_info["frame0"]
    rosTf.add (topic_info["frame0"], topic_info["frame1"], signame)
    for s in topic_info['signalGetters']:
        plug (rosTf.signal(signame), s())
    print (topic_info["frame1"], "wrt", topic_info["frame0"], "plugged to", signame, ', ', len(topic_info['signalGetters']), 'times')

def _handleHppJoint (name,topic_info,rosSubscribe,rosTf):
    if topic_info["velocity"]: topic = "velocity/op_frame"
    else:                      topic = "op_frame"
    ti = dict(topic_info)
    ti["topic"] = "/hpp/target/" + topic + '/' + topic_info['hppjoint']
    _defaultHandler (name,ti,rosSubscribe,rosTf)

def _handleHppCom (name,topic_info,rosSubscribe,rosTf):
    if topic_info["velocity"]: topic = "velocity/com"
    else:                      topic = "com"
    ti = dict(topic_info)
    if topic_info['hppcom'] == "":
        ti["topic"] = "/hpp/target/" + topic
    else:
        ti["topic"] = "/hpp/target/" + topic + '/' + topic_info['hppcom']
    _defaultHandler (name,ti,rosSubscribe,rosTf)

_handlers = {
        "hppjoint": _handleHppJoint,
        "hppcom": _handleHppCom,
        "tf_listener": _handleTfListener,
        "default": _defaultHandler,
        }
