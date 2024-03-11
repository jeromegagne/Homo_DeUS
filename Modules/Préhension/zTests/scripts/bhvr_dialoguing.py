#! /usr/bin/env python
# -*- coding: utf-8 -*-
import re
import os
import rospy
import actionlib
import traceback
import random as rand
from threading import Timer
import speech_recognizer.SpeechRecognizer as sr
import pal_interaction_msgs.msg
from custom_msgs.msg import ttsActionAction, ttsActionGoal
import xml.etree.ElementTree as ET
from enum import Enum
from std_msgs.msg import String, Bool
import HomoDeUS_common_py.HomoDeUS_common_py as common

REPEAT_MAX = 2

class Dialog_state(Enum):
    begin = 1
    listen = 2
    confirmation = 3
    done = 4

class Dialoguing_module:
    """
    This class provides a sort of chatbot so the robot can have fluid interaction with people
    """

    def __init__(self, language = "en"):
        if language == 'fr':
            dialog_context_xml_path=os.path.join(os.path.dirname(__file__), '../../homodeus_common/data_folder/dialog_context_fr.xml') 
            self.language = 'fr-CA'
        else:
            dialog_context_xml_path=os.path.join(os.path.dirname(__file__), '../../homodeus_common/data_folder/dialog_context.xml') 
            self.language = 'en-US'
        self.dialog_context = ET.parse(dialog_context_xml_path).getroot()
        
        
        # initialisation of global class variables
        self.selected_context =  ""
        self.repeat = 0     
        self.relevant_info = ""
        self.reach_deadline = False
        self.interrupt = False
        self.timer = None
        self.dialoguing_now = False
        self.dialog_state = Dialog_state.begin

        # the code works differently if it's run on the robot or not
        #param_name = rospy.search_param('on_robot')
        self.on_robot = rospy.get_param('on_robot',False)

        

        # Goal input
        self.input_bhvr_goal_context = rospy.Subscriber("/bhvr_input_goal_dialContext",data_class = String, callback=self.set_context_Cb,queue_size=10)

        # Interrupt method
        self.input_bhvr_interrupt = rospy.Subscriber("/bhvr_dialog_interrupt", data_class = Bool, callback=self.interrupt_cb, queue_size=5)
        
        # Output
        self.output_bhvr_result = rospy.Publisher("/bhvr_output_res_dialBool", Bool, queue_size=10)
        self.output_bhvr_relevant = rospy.Publisher("/bhvr_output_res_dialRelevant", String, queue_size=10)

        # dialog only use tts_server when it's run on the robot
        self.connect_to_tts_server()
        
        self.speech_recognizer = sr.SpeechRecognizer(self.language)
        
        
    def connect_to_tts_server(self):
        """
        This method connects the node to the tts_server so it can later send sentence to be pronounce by the robot
        """
        if self.on_robot:
            self.output_bhvr_command = actionlib.SimpleActionClient("tts", pal_interaction_msgs.msg.TtsAction)
        else:
            self.output_bhvr_command = actionlib.SimpleActionClient("tts", ttsActionAction)

        # wait for the action server to come up
        while(not self.output_bhvr_command.wait_for_server(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown()):
            rospy.logwarn("Waiting for the action server to come up")

    def set_context_Cb(self,context):
        """
        This method set the context of the dialog. 

        Arguments
        ---------
        context : String()
            the context to use when surfing in the XML file
        """
        # the new context is ignored if a dialog is happening right now
        if not self.dialoguing_now:
            self._reset_values()
            if self.dialog_context.getiterator(context.data):
                self.selected_context = context.data
                self.dialoguing()
            else:
                rospy.logwarn("non usable context")
        else:
            rospy.loginfo("ignoring new context")

    def _set_timer(self, time=60):
        """
        This method set a timer on the dialog so the robot doesn't stay to long in a dialog 

        Arguments
        ---------
        time : int/string
            time in seconds before reaching the deadline of the dialog
        """
        self.reach_deadline = False
        if self.timer is not None:
            self.timer.cancel()
        # time = 0 is a way to stop the timer
        if time == 0:
            return
        self.timer = Timer(int(time), self.timeout_cb)
        self.timer.start()
    
    def timeout_cb(self):
        """
        This method send an IMP_ON event.type following the timer reaching it's set time.
        """
        rospy.logdebug("reaching deadline")
        self.reach_deadline = True



    def talking(self,TtsText):
        """
        This method use the tts server to send sentence robot has to say. 

        Arguments
        ---------
        Tts : string
            the sentence to be said by the robot
        """
        # No goals are send to the tts server if the code do not run on the robot
        if self.on_robot:
            goal = pal_interaction_msgs.msg.TtsGoal()
            goal.rawtext.lang_id = self.language
            goal.rawtext.text = TtsText
        else:
            goal = ttsActionGoal()
            goal.lang_id = self.language
            goal.text = TtsText

        self.output_bhvr_command.send_goal_and_wait(goal=goal)

        # rospy.logwarn(TtsText)

    def dialoguing(self):
        """
        This method contains the main logic of this class. It follows the same pattern at each new information it gets
        from the Google API
        """
        context =self.selected_context

        #to make sure a new context doesn't begin a new dialog in an other thread
        self.dialoguing_now = True

        # dialog should run until reaching state done
        while self.dialog_state != Dialog_state.done and not self.interrupt:
            if self.dialog_state == Dialog_state.begin:
                self.talking(self._select_talking_text(self.selected_context,self.dialog_state.name))
                self._increasing_state()
                context =self.selected_context
                # set a timer so the discussion does not wander
                self._set_timer(self.dialog_context.getiterator(context)[0].get('timer'))
            listen_text = common.no_caps_and_ponctuation(self.speech_recognizer.speech_to_text())
            rospy.logdebug(listen_text)

            if self.dialog_state == Dialog_state.confirmation:
                context =  'confirmation'

            # look for relevant words in the context of the dialogue
            relevant_infos = self._get_relevant_info(context,listen_text)
        
            talking_state = self._update_talking_state(relevant_infos[0])
            if not self.interrupt:
                self._answer_client(context, talking_state, relevant_infos)

                self._update_dialog_state(talking_state,relevant_infos[0])

        if not self.interrupt:
            #what happens once the dialog done
            self._send_relevant_info(self.relevant_info) 
        else:
            self.talking("Sorry, it seems like I have urgent buisness to attend. Have a great day!")
            self._send_relevant_info("interruption")
            self.interrupt = False
        self._reset_values()
        self.dialoguing_now = False
        self.selected_context = ""
        


    def _get_relevant_info(self,context,client_answer):
        """
        This method compare the client answer with the xml file to find relevant info that could
        be usable for the next step in the scenario

        Arguments
        ---------
        context : string
            the context to look for in the xml file
        client_answer : string
            what the robot heard from the client
        """
        relevant_info = ""
        prefix = ""
        suffix = ""
        xml_path_to_search = "./"+context+"/listen/*" 
        #look for all the answer in the listen tag of the context
        for answer in self.dialog_context.findall(xml_path_to_search):
            # look for a match with the client answer. every word of the xml have to be matched to be true
            if set(answer.text.split()).issubset(client_answer.split()):
                if relevant_info:
                    relevant_info+= " "
                # every answer found in the xml is related to a relevant info to get
                relevant_info += answer.get('rel_info')
                # if there is no prefix or suffix tag, we use an empty string
                prefix = str(answer.get('prefix') or '')
                suffix = str(answer.get('suffix') or '')
        #usable info are only get at the listen state. no data are used at the confirmation state
        if self.dialog_state == Dialog_state.listen:
            self.relevant_info = self._trim_nothing(relevant_info)
        return relevant_info, prefix, suffix

    def _trim_nothing(self, relevant_info):
        """
        This method looks for 'nothing' in the relevant_info parameter. If there are other
        relevant info it means 'nothing' should be ignored. 

        Arguments
        ---------
        relevant_info : string
            the info get from the xml
        """
        relevant_list = relevant_info.split()
        if 'nothing' in relevant_list:
            if len(relevant_list) > 1:
                for info in relevant_list:
                    if info == 'nothing':
                        relevant_list.remove(info)
                relevant_info = ' '.join(map(str, relevant_list))
        return relevant_info

    def _update_talking_state(self,relevant_info):
        """
        This method updates the talking state so answer_client function knows which tag to use
        in the xml to find which answer the robot should say 

        Arguments
        ---------
        relevant_info : string
            the info get from the xml when listening
        """
        talking_state = ""
        if relevant_info:
            if relevant_info == 'nothing':
                talking_state="nothing"
            else:
                talking_state="feedback"
        # when in state of confirmation, robot can repeat more the REPEAT_MAX, but it still has a time deadline
        elif (self.repeat > REPEAT_MAX and self.dialog_state !=Dialog_state.confirmation) or self.reach_deadline:
            talking_state="deadline"
        else:
            talking_state="repeat"
        return talking_state
        

    def _update_dialog_state(self,talking_state,relevant_info):
        """
        This method updates the dialog state so _get_relevant_info knows which context to use to find informations
        in the XML. It also allows the dialog to end or to restart when needed

        Arguments
        ---------
        talking_state : string
            the talking state of the next answer. It tells where the dialog is going
        relevant_info : string
            Only used if in confirmation state to see if it's negative
        """
        if talking_state == 'feedback':
            if self.dialog_state == Dialog_state.confirmation:
                if 'negative' in relevant_info:
                    self._reset_values()
                else:
                    self._increasing_state()
            else:
                self._increasing_state()
                self._set_timer(self.dialog_context.getiterator('confirmation')[0].get('timer'))

        elif talking_state == 'repeat':
            self.repeat+=1
        elif talking_state == 'nothing' or talking_state == 'deadline':
            self.dialog_state = Dialog_state.done
            self.relevant_info = ""

    def _reset_values(self):
        """ 
        This method is used to reset the value related to the dialog state
        """
        self.dialog_state = Dialog_state.begin
        self.repeat = 0
        self.reach_deadline = False
        self._set_timer(0)
        self.relevant_info = ""

    def _increasing_state(self):
        """ 
        This method is used to increase the actual state in the dialogue
        """
        self.dialog_state = Dialog_state(self.dialog_state.value + 1)
        self.repeat = 0
    
    def _answer_client(self,context, talking_state,relevant_infos):
        """ 
        This method is used to create the sentence the robot will tell to the client
           
        Arguments
        ---------
        context : string
            the context use in the xml so it knows where to look for xml (global context)
        talking_state : string
            the talking state so it knows where to look for in the xml (sub context)
        relevant_infos: string
            relevant infos are repeated by the robots to make sure he understood well
        """
        talking_text = ""
        rel_info = self.relevant_info
        if talking_state == "feedback":       
            if self.selected_context == "menu_selection":
                rel_info = rel_info.replace(" "," and ")
            if self.selected_context == "scenario_selection":
                rel_info = rel_info.replace("_"," ")
            if context == 'confirmation' and 'negative' in relevant_infos[0]:
                return
            talking_text = self._select_talking_text(context,talking_state) + \
                relevant_infos[1] + rel_info + relevant_infos[2] 
        else:
            talking_text = self._select_talking_text(context,talking_state)
        self.talking(talking_text)

    def _send_relevant_info(self,relevant_info):
        """ 
        This method send the relevant informations get from the dialog to the topic
        bhvr_output_res_dialRelevant and send a True to the topic bhvr_output_res_dialBool
        so to alert the dialog is done
           
        Arguments
        ---------
        relevant_info: string
            relevant informations that will be use by another node
        """
        self.output_bhvr_relevant.publish(relevant_info)
        self.output_bhvr_result.publish(True)

    def _select_talking_text(self,global_context, specific_context):
        """ 
        This method selects randomly the answer listed in the XML based on the talking_state 
        and the context given
           
        Arguments
        ---------
        global_context: string
            the context of the dialog
            Ex. 'menu_selection', 'confirmation'
        specific_context:
            where to look for in the global context. 
            Ex. look for the tag 'repeat'
        """
        global_context_iter = self.dialog_context.getiterator(global_context)
        outlist = global_context_iter[0].findall("./"+specific_context+"/*")
        return outlist[rand.randint(0,(len(outlist)-1))].text

    def interrupt_cb(self,interrupt):
        rospy.logwarn("Dialog being interrupted")
        if self.dialoguing_now and interrupt.data:
            self.interrupt = True


                

    def node_shutdown(self):
        """
        This method informs the developper about the shutdown of this node
        """
        self.output_bhvr_command.cancel_goal()
        rospy.loginfo("have been shutdown")


if __name__ == "__main__":
    """
    Starts a node with the name of the file and calls
    the transform function. It only shutdown if an extern event ask for it
    """
    try:
        rospy.init_node(common.get_file_name(__file__))
        lang = rospy.get_param('lang','en')
        node = Dialoguing_module(lang)
        rospy.on_shutdown(node.node_shutdown)
        rospy.spin()

    except Exception:
        rospy.logerr(traceback.format_exc())

