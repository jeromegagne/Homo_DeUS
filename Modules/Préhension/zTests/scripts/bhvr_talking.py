#! /usr/bin/env python
import rospy
import time
import actionlib
import traceback
import pal_interaction_msgs.msg
from custom_msgs.msg import ttsActionAction, ttsActionGoal, ttsActionResult
from std_msgs.msg import String, Bool
import HomoDeUS_common_py.HomoDeUS_common_py as common

class Talking_module:
    """
    This class provide control to the robot's head as an actionlib server
    """
    def __init__(self, language = "en-GB", text="Welcome my friends"):
        """
        This method initializes Talking_module by connecting to the relevant input and output
        It can than receives a goal, sending a command and publishes when its goal is achievec

        Arguments
        ---------
        language : String
            Which language should the robot use when talking
        text: String
            What the robot should say 
        """
        if language == 'fr':
            self.language = 'fr-FR'
        else:
            self.language = 'en-GB'
            
        self.talking_text = text
        # Goal input
        self.input_bhvr_goal = rospy.Subscriber("/bhvr_input_goal_talking",data_class=String,callback=self.action_Cb,queue_size=10)
        self.input_bhvr_interrupt = rospy.Subscriber("/bhvr_talking_interrupt",data_class=Bool,callback=self.interrupt_cb,queue_size=5)

        #look if being test on robot or computer 
        #param_name = rospy.search_param('on_robot')
        self.on_robot = rospy.get_param('on_robot',False)
        
        # Output
        self.output_bhvr_result = rospy.Publisher("/bhvr_output_res_talking", Bool, queue_size=10)
        self.output_bhvr_muteSpeech = rospy.Publisher("/bhvr_output_isTalking", Bool, queue_size=5)

        if self.on_robot:
            self.output_bhvr_command = actionlib.SimpleActionClient("tts", pal_interaction_msgs.msg.TtsAction)
        else:
            self.output_bhvr_command = actionlib.SimpleActionClient("tts", ttsActionAction)
        # wait for the action server to come up
        while(not self.output_bhvr_command.wait_for_server(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown()):
            rospy.loginfo("Waiting for the action server to come up")
        
        rospy.loginfo("Connection to server done")

    def action_Cb(self, TtsText):
        """
        This method converts text to speech using the proper action server
        made by PAL Robotics.

        Arguments
        ---------
        TtsText : String
            String mentionning what to say by the robot
        """
        self.output_bhvr_muteSpeech.publish(True)
        if self.on_robot:
            goal = pal_interaction_msgs.msg.TtsGoal()
            goal.rawtext.lang_id = self.language
            if TtsText == "":
                goal.rawtext.text= self.talking_text
            else:
                goal.rawtext.text = TtsText.data
        
        else:
            goal = ttsActionGoal()
            goal.lang_id = self.language
            if TtsText == "":
                goal.rawtext.text= self.talking_text
            else:
                goal.text = TtsText.data
        self.output_bhvr_command.send_goal(goal=goal,done_cb=self.goal_achieve_Cb)
        
        rospy.loginfo(TtsText.data)

    def goal_achieve_Cb(self, _, __):
        """
        This method publishes a confirmation the goal received was achieved
        """
        self.output_bhvr_result.publish(True)

    def interrupt_cb(self,interrupt):
        rospy.loginfo("Interrupting talking bhvr")
        if interrupt.data:
            self.output_bhvr_command.cancel_all_goals()

    def node_shutdown(self):
        """
        This method cancel goal if their is a sudden shutdown. It also informs by a log that the node was shutdown
        """
        self.output_bhvr_command.cancel_goal()
        rospy.loginfo(self,"have been shutdown")


if __name__ == "__main__":
    """
    It creates a node and starts the tts server in it and connects to all topics needed for this module
    """
    try:
        rospy.init_node(common.get_file_name(__file__))
        lang = rospy.get_param('lang', 'en')
        node = Talking_module(lang)
        rospy.on_shutdown(node.node_shutdown)
        rospy.spin()

    except Exception:
        rospy.logerr(traceback.format_exc())
