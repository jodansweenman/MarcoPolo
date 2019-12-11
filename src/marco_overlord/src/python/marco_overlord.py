#!/usr/bin/env python

from statemachine.exceptions import TransitionNotAllowed, InvalidDefinition
from statemachine import StateMachine, State
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String
from speech_node.msg import SynthRq, PlayRq
from transcription_node.msg import TranscriptionResult

class MPOverlord(StateMachine):

    # List of States
    initializing = State('Initializing', initial=True)
    search_mapping = State('Searching - Mapping')
    search_paused = State('Searching - Paused')
    ballfound_mapping = State("Ball Found - Mapping")
    ballfound_paused = State("Ball Found - Paused")
    ballfound_pursuing = State("Ball Found - Pursuing")

    # These are all callbacks which play audio upon entering state
    def on_enter_ballfound_mapping(self):
        rospy.loginfo("Now in %s" % self.ballfound_mapping.name)
        playrq = PlayRq(synth_name=self.ballfound_mapping.identifier)
        self.playrq_pub.publish(playrq)
        return
    
    def on_enter_ballfound_paused(self):
        rospy.loginfo("Now in %s" % self.ballfound_paused.name)
        playrq = PlayRq(synth_name=self.ballfound_paused.identifier)
        self.playrq_pub.publish(playrq)
        return
    
    def on_enter_ballfound_pursuing(self):
        rospy.loginfo("Now in %s" % self.ballfound_pursuing.name)
        playrq = PlayRq(synth_name=self.ballfound_pursuing.identifier)
        self.playrq_pub.publish(playrq)
        return
    
    def on_enter_search_mapping(self):
        rospy.loginfo("Now in %s" % self.search_mapping.name)
        playrq = PlayRq(synth_name=self.search_mapping.identifier)
        self.playrq_pub.publish(playrq)
        return
    
    def on_enter_search_paused(self):
        rospy.loginfo("Now in %s" % self.search_paused.name)
        playrq = PlayRq(synth_name=self.search_paused.identifier)
        self.playrq_pub.publish(playrq)
        return

    # Events which will trigger transitions
    begin_mapping = initializing.to(search_mapping)
    found_ball = search_mapping.to(ballfound_mapping)
    
    ball_found_pause = ballfound_mapping.to(ballfound_paused)
    ball_found_unpause = ballfound_paused.to(ballfound_mapping)
    
    searching_pause = search_mapping.to(search_paused)
    searching_unpause = search_paused.to(search_mapping)

    # Can only pursue the ball if requested from the paused state
    bf_pursue_from_pause = ballfound_paused.to(ballfound_pursuing)
    
    def _go_to_state(self, state_to_goto):
        transtocall = None
        for trans in self.allowed_transitions:
            for dest in trans.destinations:
                if dest == self.ballfound_pursuing:
                    transtocall = trans
        if transtocall is None:
            raise InvalidDefinition()
        else:
            transtocall()

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
        super(MPOverlord, self).__init__()

        rospy.loginfo('Initializing Marco Polo Overlord')

        # Default query texts
        self.qtexts = {'wake': 'hey marco', 'where': 'where is the', 'fetch': 'go get the'}

        # Registering all the subscribers and publishers
        rospy.loginfo('Registering as publisher for /mpstate/synth_rq')
        self.synrq_pub = rospy.Publisher('mpstate/synth_rq', SynthRq, queue_size=10, latch=True)

        rospy.loginfo('Registering as publisher for /mpstate/play_rq')
        self.playrq_pub = rospy.Publisher('mpstate/play_rq', PlayRq, queue_size=10, latch=True)

        rospy.loginfo('Registering as publisher for /mpstate/pause_rq')
        self.pauserq_pub = rospy.Publisher('mpstate/pause_rq', Bool, queue_size=10)

        rospy.loginfo('Registering as publisher for /marco/pursue_cube')
        self.pursuerq_pub = rospy.Publisher('marco/pursue_cube', String, queue_size=10)

        rospy.loginfo('Registering as subscriber of /geopoint')
        self.ballxy_sub = rospy.Subscriber('geopoint', Point, callback=self.ballxy_received)

        rospy.loginfo('Registering as subscriber of /mpstate/transcription')
        self.query_sub = rospy.Subscriber('mpstate/transcription', TranscriptionResult,
                                          callback=self.query_received)

        self.ball_xy_localframe = None

        rospy.sleep(1)

        self._rq_gen_state_trans_msgs()
        self._rq_gen_default_message()

        rospy.sleep(1)

        self.begin_mapping()

        return

    def _rq_gen_state_trans_msgs(self):
        rospy.loginfo('Requesting generation of state messages')
        for state in self.states:
            rospy.sleep(2)
            print(state.name)
            synrq = SynthRq(synth_name=state.identifier,
                            synth_text="Now in %s" % state.name,
                            force_resynth=False)
            self.synrq_pub.publish(synrq)

    def _rq_gen_default_message(self):
        rospy.loginfo('Requesting generation of default message')
        defaultsynrq = SynthRq(synth_name='ball_not_found',
                               synth_text="I haven't found the ball yet!",
                               force_resynth=False)
        self.synrq_pub.publish(defaultsynrq)

        return

    def query_received(self, data):
        query_string = data.transcribed_text
        conf_in_query = data.confidence
        rospy.loginfo("Received query: '%s' with confidence: %f" % (query_string, conf_in_query))
        print("State: %s" % self.current_state_value)

        if self.qtexts['wake'] in query_string.lower():
            rospy.loginfo("Beginning of query for Marco")
            # Synthesize the location of the ball since next question is probably
            # related to the ball

            if 'ballfound' in self.current_state_value:
                full_ball_text = self._generate_relative_dist_str()
            
                ballsynrq = SynthRq(synth_name='ballfound_loc',
                                    synth_text=full_ball_text,
                                    force_resynth=True)
                self.synrq_pub.publish(ballsynrq)

            #Pause the robot
            try:
                self.pause()
            except TransitionNotAllowed:
                rospy.logwarn("Already paused, not transitioning")
            else:
                self.pauserq_pub.publish(Bool(data=True))

        elif self.qtexts['where'] in query_string.lower():
            rospy.loginfo("Received query is related to the ball")
            playrq = PlayRq()
            if 'ballfound' in self.current_state_value:
                playrq.synth_name = 'ballfound_loc'
            else:
                playrq.synth_name = 'ball_not_found'
            
            self.playrq_pub.publish(playrq)

            #Resume the movement of the robot

            try:
                self.resume()
            except TransitionNotAllowed:
                rospy.logwarn("Not allowed to unpause from this state")
            else:
                self.pauserq_pub.publish(Bool(data=False))

        elif self.qtexts['fetch'] in query_string.lower():
            rospy.loginfo("Received request to get the cube")
            try:
                self.bf_pursue_from_pause()
            except TransitionNotAllowed:
                rospy.logwarn("Cannot pursue ball, it's not found yet!")
                playrq = PlayRq()
                playrq.synth_name = 'ball_not_found'

                self.playrq_pub.publish(playrq)
            else:
                # Sending an empty string pursue request is enough to move into position
                pursuerq = String()
                self.pursuerq_pub.publish(pursuerq)
                playrq = PlayRq()
                playrq.synth_name = self.ballfound_pursuing.identifier
        
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

        full_ball_text = 'The ball is located %.1f meters %s and %.1f meters %s.' % \
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
            self.found_ball()
        except TransitionNotAllowed:
            rospy.logwarn("Ball already found, not transitioning")

        return

def main():

    rospy.init_node(name='marco_overlord')
    mp_ol = MPOverlord()

    rospy.spin()

    return

if __name__ == "__main__":
    main()
