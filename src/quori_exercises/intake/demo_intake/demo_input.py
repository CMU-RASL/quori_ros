#Current iteration of the terminal inputs
#This is a script that allows the user to input the key and key_specific to get the corresponding message from the intake_messages.py file
from demo_messages import *



def get_key():
    try:
        section_num_input=input("What section are you on? \n 1. Introduction \n 2.Edward_Evaluation \n 3. Edith_Evaluation \n 4. Clara_Evaluation \n 5. Exercise explanation \n 6. Post_set_1 \n 7. Post_set_2 \n 8. Fillers \n 9. Fall back \n q to quit \n")
        
        if section_num_input == "1": key="Introduction"
        elif section_num_input == "2": key= "Edward_Evaluation"
        elif section_num_input == "3": key= "Edith_Evaluation"
        elif section_num_input == "4": key="Clara_Evaluation"
        elif section_num_input == "5": key= "Exercise explanation"
        elif section_num_input == "6": key= "Post_set_1"
        elif section_num_input == "7": key= "Post_set_2"
        elif section_num_input == "8": key= "Fillers"
        elif section_num_input == "9": key= "Fall back"
        elif section_num_input == "q": key= "quit"
        return key
        
        
    except:
        print("Invalid input try again")
        return get_key()
        
    
def get_key_intro():
    response_num=input("1. Greeting \n 2.Frida \n 3.Great \n 4. me \n b to go back \n")
    try:
        if response_num == "1": key_specific="Greeting"
        elif response_num== "2": key_specific="Frida"
        elif response_num == "3": key_specific="Great"
        elif response_num == "4": key_specific="me"
        elif response_num == "b": key_specific="back"
        return key_specific
    except:
        print("Invalid input")
        return get_key_intro()
    

def get_edith():
    response_num=input("1. Feel \n 2. Excellent \n b to go back \n")
    try:
        if response_num == "1": key_specific="Feel"
        elif response_num == "2": key_specific="Excellent"
        elif response_num == "b": key_specific="back"
        return key_specific
    except:
        print("Invalid input")
        return get_edith()
    

def get_edward():
    response_num=input("1. Feel \n 2. Sleep \n 3. Thanks \n b to go back \n")
    try:
        if response_num == "1": key_specific="Feel"
        elif response_num== "2": key_specific="Sleep"
        elif response_num== "3": key_specific="Thanks"
        elif response_num == "b": key_specific="back"
        return key_specific
    except:
        print("Invalid input")
        return get_edward()
    

def get_clara():
    response_num=input("1. Feel \n 2. Breakfast \n 3. Thanks \n b to go back \n")
    try:
        if response_num== "1": key_specific="Feel"
        elif response_num == "2": key_specific="Breakfast"
        elif response_num == "3": key_specific="Thanks"
        elif response_num == "b": key_specific="back"
        return key_specific
    except:
        print("Invalid input")
        return get_clara()
    

def get_key_exercise():
    response_num=input("1. Demo explanation \n b to go back \n")
    try:
        if response_num == "1": key_specific="Demo explanation"
        elif response_num == "b": key_specific="back"
        return key_specific
    except:
        print("Invalid input")
        return get_key_exercise()

def get_Post1():
    response_num=input("1. Feedback \n 2. good \n 3. great \n 4. joke \n 5. Filler_park \n 6. Filler_mountain \n 7. Next_set \n b to go back \n")
    try:
        if response_num == "1": key_specific="Feedback"
        elif response_num=="2": key_specific="good"
        elif response_num=="3": key_specific="great"
        elif response_num== "4": key_specific="joke"
        elif response_num== "5": key_specific="Filler_park"
        elif response_num== "6": key_specific="Filler_mountain"
        elif response_num== "7": key_specific="Next_set"
        elif response_num== "b": key_specific="back"
        return key_specific
    except:
        print("Invalid input")
        return get_Post1()

def get_Post2():
    response_num=input("1. Feedback \n 2. good \n 3. great \n b to go back \n")
    try:
        if response_num == "1": key_specific="Feedback"
        elif response_num=="2": key_specific="good"
        elif response_num=="3": key_specific="great"
        elif response_num== "b": key_specific="back"
        return key_specific
    except:
        print("Invalid input")
        return get_Post2()
     
def get_key_fall_back():
    response_num=input("1. No answer \n 2. Repeat  \n b to go back \n")
    try:
        if response_num == "1": key_specific="No answer"
        elif response_num== "2": key_specific="Repeat"
        elif response_num == "b": key_specific="back"
        return key_specific
    except:
        print("Invalid input")
        return get_key_fall_back()

def get_key_fillers():
    response_num=input("1. know \n 2. yes \n 3. laugh \n 4. no \n 5. ghost \n 6. ghost1 \n b to go back \n")
    try:
        if response_num == "1": key_specific="know"
        elif response_num== "2": key_specific="yes"
        elif response_num== "3": key_specific="laugh"
        elif response_num== "4": key_specific="no"
        elif response_num== "5": key_specific="ghost"
        elif response_num== "6": key_specific="ghost1"
        elif response_num == "b": key_specific="back"
        return key_specific
    except:
        print("Invalid input")
        return get_key_fillers()

def quit():
    return "quit"


def get_terminal_input(key):
   #key=get_key()
    if key == "Introduction":
        key_specific=get_key_intro()
    elif key == "Edward_Evaluation":
        key_specific=get_edward()
    elif key == "Edith_Evaluation":
        key_specific=get_edith()
    elif key == "Clara_Evaluation":
        key_specific=get_clara()
    elif key == "Exercise explanation":
        key_specific=get_key_exercise()
    elif key == "Post_set_1":
        key_specific=get_Post1()
    elif key == "Post_set_2":
        key_specific=get_Post2()
    elif key == "Fillers":
        key_specific=get_key_fillers()
    elif key == "Fall back":
        key_specific=get_key_fall_back()
    elif key == "quit":
        key_specific=quit()

    return key_specific


    
    

    