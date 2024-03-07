#!/usr/bin/env python3
import roslib
import rospy
import smach
import smach_ros
from std_msgs.msg import Float64MultiArray, String
import numpy as np

class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_study'])

    def execute(self, userdata):
        rospy.loginfo('Executing state START - Pausing for 1 sec')
        # sleep for duration
        d = rospy.Duration(1, 0)
        rospy.sleep(d)
        return 'continue_study'


class Consent(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_study', 'exit_study'])
        self.consent_result = 1

    def execute(self, userdata):
        rospy.loginfo('Executing state CONSENT - Pausing for 1 sec')
        # sleep for duration
        d = rospy.Duration(1, 0)
        rospy.sleep(d)
        if self.consent_result == 1:
            rospy.loginfo('CONSENT completed')
            return 'continue_study'
        else:
            rospy.loginfo('CONSENT not completed')
            return 'exit_study'
        

class Training(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_study'])

    def execute(self, userdata):
        rospy.loginfo('Executing state TRAINING - Pausing for 1 sec')
        # sleep for duration
        d = rospy.Duration(1, 0)
        rospy.sleep(d)
        return 'continue_study'

class ExerciseGroup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_study', 'begin_exercise'],
                                    input_keys=['sm_exercise_order', 'sm_set_counter', 'sm_sets_per_exercise'],
                                    output_keys=['sm_current_exercise'])

        
    def execute(self, userdata):
        rospy.loginfo('Executing state EXERCISEGROUP')
        for exercise_index, (exercise, sets_complete) in enumerate(zip(userdata.sm_exercise_order, userdata.sm_set_counter)):
            if sets_complete < userdata.sm_sets_per_exercise:
                userdata.sm_current_exercise = [exercise_index, exercise, sets_complete+1]
                return 'begin_exercise'
            else:
                rospy.loginfo('Exercise {} complete'.format(exercise))
        
        return 'continue_study'


class Exercise(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['begin_rest'],
                                    input_keys=['sm_current_exercise', 'sm_set_counter', 'sm_set_counter', 'sm_sets_per_exercise', 'sm_exercise_time'],
                                    output_keys=['sm_set_counter'])
        self.quori_body_face_pub = rospy.Publisher("quori_body_face", Float64MultiArray, queue_size=2)
        self.quori_sound_pub = rospy.Publisher('quori_sound', String, queue_size=2)

    def execute(self, userdata):
        rospy.loginfo('Executing state EXERCISE {}/{} ({}) - Set {}/{}'.format(userdata.sm_current_exercise[0]+1, 
                                                                            len(userdata.sm_set_counter), 
                                                                            userdata.sm_current_exercise[1],
                                                                            userdata.sm_current_exercise[2],
                                                                            userdata.sm_sets_per_exercise))
                
        last_message = rospy.get_rostime()
        robot_message = "Let us start set %s out of %s of %s" % (userdata.sm_current_exercise[2],
                                                                    userdata.sm_sets_per_exercise, userdata.sm_current_exercise[1])
        rospy.loginfo("Robot will say: {}".format(robot_message))
        self.quori_sound_pub.publish(robot_message)
        
        start = rospy.get_rostime()
        d = rospy.Duration(userdata.sm_exercise_time, 0)

        while start + d > rospy.get_rostime():
            if last_message + rospy.Duration(10, 0) < rospy.get_rostime():
                last_message = rospy.get_rostime()
                time_left = int(np.round((d - (rospy.get_rostime() - start)).to_sec()))
                robot_message = "%s seconds left on %s" % (time_left, userdata.sm_current_exercise[1])
                rospy.loginfo("Robot will say: {}".format(robot_message))
                self.quori_sound_pub.publish(robot_message)

                body_face_msg = Float64MultiArray()
                body_face_msg.data = [1, 1, 2]
                rospy.loginfo("Robot will move: {}".format(body_face_msg.data))
                self.quori_body_face_pub.publish(body_face_msg)

        return 'begin_rest'

class Rest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['end_exercise'],
                                    input_keys=['sm_current_exercise', 'sm_set_counter', 'sm_rest_between_sets', 'sm_rest_between_exercises', 'sm_sets_per_exercise'],
                                    output_keys=['sm_current_exercise'])
        self.quori_body_face_pub = rospy.Publisher("quori_body_face", Float64MultiArray, queue_size=2)
        self.quori_sound_pub = rospy.Publisher('quori_sound', String, queue_size=2)
        
    def execute(self, userdata):
        rospy.loginfo('Executing state EXERCISE {}/{} ({}) - Set {}/{}'.format(userdata.sm_current_exercise[0]+1, 
                                                                            len(userdata.sm_set_counter), 
                                                                            userdata.sm_current_exercise[1],
                                                                            userdata.sm_current_exercise[2],
                                                                            userdata.sm_sets_per_exercise))

        #Check if last set in exercise
        if userdata.sm_set_counter[userdata.sm_current_exercise[0]] == userdata.sm_sets_per_exercise - 1:
            rest_time = userdata.sm_rest_between_exercises
        else:
            rest_time = userdata.sm_rest_between_sets
        
        d = rospy.Duration(rest_time, 0)
        rospy.loginfo('Resting for {} sec'.format(rest_time))
        robot_message = 'Rest for {} seconds'.format(rest_time)
        self.quori_sound_pub.publish(robot_message)
        
        rospy.sleep(d)
        userdata.sm_set_counter[userdata.sm_current_exercise[0]] += 1
        return 'end_exercise'




#Start with this
#Start -> Consent -> Explanation -> Exercise 1 -> Rest -> Repeat 3 -> Done

def main():
    rospy.init_node('main_state_machine')

    # Create the top level SMACH state machine
    sm_main = smach.StateMachine(outcomes=['complete'])
    sm_main.userdata.sm_exercise_order = ['bicep curls', 'lateral raises']
    sm_main.userdata.sm_set_counter = [0, 0]
    sm_main.userdata.sm_sets_per_exercise = 3
    sm_main.userdata.sm_rest_between_sets = 30
    sm_main.userdata.sm_rest_between_exercises = 30
    sm_main.userdata.sm_exercise_time = 30
    sm_main.userdata.sm_current_exercise = [-1, '', -1] #Exercise number, exercise name, set number

    # Open the container
    with sm_main:

        smach.StateMachine.add('START', Start(),
                               transitions={'continue_study':'CONSENT'})

        smach.StateMachine.add('CONSENT', Consent(),
                               transitions={'continue_study':'TRAINING', 'exit_study': 'complete'})
        
        smach.StateMachine.add('TRAINING', Training(),
                               transitions={'continue_study':'EXERCISEGROUP'})

        smach.StateMachine.add('EXERCISEGROUP', ExerciseGroup(),
                               transitions={'continue_study':'complete', 'begin_exercise': 'EXERCISE'})

        smach.StateMachine.add('EXERCISE', Exercise(),
                               transitions={'begin_rest': 'REST'})
        
        smach.StateMachine.add('REST', Rest(),
                               transitions={'end_exercise': 'EXERCISEGROUP'})

       
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm_main, '/')
    sis.start()

    # Execute the state machine
    outcome = sm_main.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()