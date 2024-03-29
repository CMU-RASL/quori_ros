#Current iteration of the terminal inputs
#This is a script that allows the user to input the key and key_specific to get the corresponding message from the intake_messages.py file
from intake_messages import *



def get_key():
    try:
        section_num_input=input("What section are you on? \n 1. Introduction \n 2.Consent \
                    \n 3. Evaluation \n 4. Exercise explanation  \n 5. Coach Type \n 6. Start exercise \n 7. Fall back \n 8. quit \n")
        
        if section_num_input == "1": key="Introduction"
        elif section_num_input == "2": key= "Consent"
        elif section_num_input == "3": key= "Evaluation"
        elif section_num_input == "4": key="Exercise explanation"
        elif section_num_input == "5": key= "Coach type"
        elif section_num_input == "6": key= "Start exercise"
        elif section_num_input == "7": key= "Fall back"
        elif section_num_input == "8": key= "quit"
        return key
        
        
    except:
        print("Invalid input try again")
        return get_key()
        
    
def get_key_intro():
    response_num=input("1. Greeting \n 2. Fun \n 3. Response positive \n 4. Response negative \n 5. Good \n 6. back \n")
    try:
        if response_num == "1": key_specific="Greeting"
        elif response_num== "2": key_specific="Fun"
        elif response_num == "3": key_specific="Response positive"
        elif response_num == "4": key_specific="Response negative"
        elif response_num == "5": key_specific="Good"
        elif response_num == "6": key_specific="back"
        return key_specific
    except:
        print("Invalid input")
        return get_key_intro()
    

def get_key_consent():
    response_num=input("1.Start consent form \n 2. Part 1 \n 3. Part 2 \n 4. Part 3 \n 5. Part 4 \n 6. Explanation \n 7. Time \n 8. back \n")
    try:
        if response_num == "1": key_specific="Start consent form"
        elif response_num== "2": key_specific="Part 1"
        elif response_num == "3": key_specific="Part 2"
        elif response_num == "4": key_specific="Part 3"
        elif response_num == "5": key_specific="Part 4"
        #elif response_num == "6": key_specific="Part 5"
        elif response_num == "6": key_specific="Explanation"
        elif response_num == "7": key_specific="Time"
        elif response_num == "8": key_specific="back"
        return key_specific
    except:
        print("Invalid input")
        return get_key_consent()
    

def get_key_evaluation():
    response_num=input("1. Pain \n 2. Energy Level \n 3. back \n")
    try:
        if response_num == "1": key_specific="Pain"
        elif response_num== "2": key_specific="Energy Level"
        elif response_num == "3": key_specific="back"
        return key_specific
    except:
        print("Invalid input")
        return get_key_evaluation()
    

def get_key_exercise():
    response_num=input("1. Start explanation \n 2. Explain exercise routine \n 3. Dumbbells \n 4. Questions \n 5. back \n")
    try:
        if response_num == "1": key_specific="Start explanation"
        elif response_num== "2": key_specific="Explain exercise routine"
        elif response_num == "3": key_specific="Dumbbells"
        elif response_num == "4": key_specific="Questions"
        elif response_num == "5": key_specific="back"
        return key_specific
    except:
        print("Invalid input")
        return get_key_exercise()
    

def get_key_coach_type():
    response_num=input("Ask coach type question?")
    try:
        if response_num == "": key_specific="Ask type"
        elif response_num== "quit": key_specific="quit"
        elif response_num == "back": key_specific="back"
        return key_specific
    except:
        print("Invalid input")
        return get_key_coach_type()

def get_key_start_exercise():
    response_num=input("1. Start \n 2. back \n")
    try:
        if response_num == "1": key_specific="Start"
        elif response_num== "2": key_specific="back"
        return key_specific
    except:
        print("Invalid input")
        return get_key_start_exercise()
     
def get_key_fall_back():
    response_num=input("1. No answer \n 2. Repeat \n 3. Clarify \n  4. back \n")
    try:
        if response_num == "1": key_specific="No answer"
        elif response_num== "2": key_specific="Repeat"
        elif response_num == "3": key_specific="Clarify"
        elif response_num == "4": key_specific="back"
        return key_specific
    except:
        print("Invalid input")
        return get_key_fall_back()

def quit():
    return "quit"


def get_terminal_input(key):
   #key=get_key()
    if key == "Introduction":
        key_specific=get_key_intro()
    elif key == "Consent":
        key_specific=get_key_consent()
    elif key == "Evaluation":
        key_specific=get_key_evaluation()
    elif key == "Exercise explanation":
        key_specific=get_key_exercise()
    elif key == "Coach type":
        key_specific=get_key_coach_type()
    elif key == "Start exercise":
        key_specific=get_key_start_exercise()
    elif key == "Fall back":
        key_specific=get_key_fall_back()
    elif key == "quit":
        key_specific=quit()



    return key_specific


    
    

    