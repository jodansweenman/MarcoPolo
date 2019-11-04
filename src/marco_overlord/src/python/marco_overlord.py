#!/usr/bin/env python

from statemachine import StateMachine, State
from statemachine.exceptions import TransitionNotAllowed
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from speech_node.msg import SynthRq, PlayRq
from transcription_node.msg import TranscriptionResult

class MPOverlord:
    def __init__(self):
        rospy.loginfo('Initializing Marco Polo Overlord')
        self.mpstate = MPState()

        self.mpstate.begin_mapping()

        rospy.loginfo('Registering as publisher for /mpstate/synth_rq')
        self.synrq_pub = rospy.Publisher('mpstate/synth_rq', SynthRq, queue_size=10, latch=True)

        rospy.loginfo('Registering as publisher for /mpstate/play_rq')
        self.playrq_pub = rospy.Publisher('mpstate/play_rq', PlayRq, queue_size=10, latch=True)

        rospy.loginfo('Registering as publisher for /mpstate/pause_rq')
        self.pauserq_pub = rospy.Publisher('mpstate/pause_rq', Bool, queue_size=10)

        rospy.loginfo('Registering as subscriber of /ballxy')
        self.ballxy_sub = rospy.Subscriber('ballxy', Point, callback=self.ballxy_received)

        rospy.loginfo('Registering as subscriber of /mpstate/transcription')
        self.query_sub = rospy.Subscriber('mpstate/transcription', TranscriptionResult,
                                          callback=self.query_received)

        self.ball_xy_localframe = None

        self._rq_gen_default_message()

        return

    def _rq_gen_default_message(self):
        rospy.loginfo('Requesting generation of default message')
        defaultsynrq = SynthRq(synth_name='ball_not_found',
                               synth_text="I haven't found the ball yet!")
        self.synrq_pub.publish(defaultsynrq)

        return

    def query_received(self, data):
        query_string = data.transcribed_text
        conf_in_query = data.confidence
        rospy.loginfo("Received query: '%s' with confidence: %f" % (query_string, conf_in_query))
        print("State: %s" % self.mpstate.current_state_value)

        if "Marco" in query_string:
            rospy.loginfo("Beginning of query for Marco")
            # Synthesize the location of the ball since next question is probably
            # related to the ball

            #Pause the robot
            try:
                self.mpstate.pause()
            except TransitionNotAllowed:
                rospy.logwarn("Already paused, not transitioning")
            else:
                self.pauserq_pub.publish(Bool(data=True))

            if 'ballfound' in self.mpstate.current_state_value:
                full_ball_text = self._generate_relative_dist_str()
            
                ballsynrq = SynthRq(synth_name='ballfound_loc',
                                synth_text=full_ball_text)
                self.synrq_pub.publish(ballsynrq)


        elif "red ball" in query_string:
            rospy.loginfo("Received query is related to the ball")
            playrq = PlayRq()
            if 'ballfound' in self.mpstate.current_state_value:
                playrq.synth_name = 'ballfound_loc'
            else:
                playrq.synth_name = 'ball_not_found'
            
            self.playrq_pub.publish(playrq)

            #TODO: resume the movement of the robot

            try:
                self.mpstate.resume()
            except TransitionNotAllowed:
                rospy.logwarn("Not allowed to unpause from this state")
            else:
                self.pauserq_pub.publish(Bool(data=False))

        else:
            rospy.loginfo("Unknown Query")

        return

    def _generate_relative_dist_str(self):

        ball_relative_loc = self.ball_xy_localframe

        # Determine text for lateral position
        if ball_relative_loc[0] > 0:
            left_right_text = 'to my right'
        else:
            left_right_text = 'to my left'

        # Determine text for longitudinal position
        if ball_relative_loc[1] > 0:
            forward_back_text = 'ahead of me'
        else:
            forward_back_text = 'behind me'

        full_ball_text = 'The ball is located %s meters %s and %s meters %s.' % \
                    (abs(ball_relative_loc[0]),
                     left_right_text,
                     abs(ball_relative_loc[1]),
                     forward_back_text)
        return full_ball_text


    # This needs to handle the global coordinates and transformation
    def ballxy_received(self, data):
        rospy.loginfo("Received value for ball location: %s" % data)

        self.ball_xy_localframe = (data.x, data.y)

        try:
            self.mpstate.found_ball()
        except TransitionNotAllowed:
            rospy.logwarn("Ball already found, not transitioning")

        return

class MPState(StateMachine):
    """
    State machine defining Marco Polo
    """
    # List of States
    initializing = State('Initializing', initial=True)
    search_mapping = State('Searching - Mapping')
    search_paused = State('Searching - Paused')
    ballfound_mapping = State("Ball Found - Mapping")
    ballfound_paused = State("Ball Found - Paused")

    # Events which will trigger transitions
    begin_mapping = initializing.to(search_mapping)
    found_ball = search_mapping.to(ballfound_mapping)
    
    ball_found_pause = ballfound_mapping.to(ballfound_paused)
    ball_found_unpause = ballfound_paused.to(ballfound_mapping)
    
    searching_pause = search_mapping.to(search_paused)
    searching_unpause = search_paused.to(search_mapping)
    
    def resume(self):
        if self.current_state == self.search_paused:
            self.searching_unpause()
        elif self.current_state == self.ballfound_paused:
            self.ball_found_unpause()        
    
    def pause(self):
        if self.current_state == self.search_mapping:
            self.searching_pause()
        elif self.current_state == self.ballfound_mapping:
            self.ball_found_pause()
            
    def __init__(self):
        super(MPState, self).__init__()

    def on_enter_search_mapping(self):
        print("Now searching for the ball ...")
        # Code to begin robot movement

    def on_enter_ballfound_mapping(self):
        print("Found the ball!  Continuing mapping ...")
        
    def on_enter_search_paused(self):
        print("Pausing search for ball ...")
    
    def on_enter_ballfound_paused(self):
        print("Pausing continous mapping.  Ball location is still known")
    

def main():

    rospy.init_node(name='marco_overlord')
    mp_ol = MPOverlord()

    rospy.spin()

    return

if __name__ == "__main__":
    main()
