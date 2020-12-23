#!/usr/bin/env python

class Tablet:
    def __init__(self, robot):
        self.display = Display(robot)
        self.speaker = Speaker(robot)

    def greeting(self):
        self.display.load_image('greetings')
        self.speaker.speak_polyglot(it_IT="Ciao", en_US="Hello")

    def help(self):
        self.display.load_image('help')
        self.speaker.speak_polyglot(it_IT="Aiuto", en_US="Help me")
 

    def make_way(self):
        self.display.load_image('make_way')
        self.speaker.speak_polyglot(it_IT="Permesso", en_US="Please make way")
 

    def warning(self):
        self.display.load_image('warning')
        self.speaker.speak_polyglot(it_IT="Per favore rispettare il distanziamento sociale", en_US="Please respect social distancing")
               


class Speaker:
    def __init__(self,robot,name='speaker',path='../../../../../Media/Audio/'):
        self.name=name
        self.path=path
        self.__robot=robot

    def play_sound(self,sound):
        self.__robot.call_service(self.name,'play_sound',self.path + sound + '.mp3', 1.0, 1.0, 0.0, False)

    def is_speaking(self):
        response = self.__robot.call_service(self.name,'is_speaking')
        return response.value

    def speak(self,text,volume=1.0):
        while(self.is_speaking()):
            pass
        self.__robot.call_service(self.name,'speak', text, volume)

    def speak_polyglot(self,it_IT=None,en_US=None,de_DE=None,es_ES=None,fr_FR=None,en_UK=None):
        for language, text in locals().items():
            if text is not None and language != 'self':
                self.__robot.call_service(self.name, 'set_language', language.replace("_","-"))
                self.speak(text)                   

class Display:
    def __init__(self,robot,name='display',path='../../../../../Media/Image/'):              
        self.name=name
        self.path=path
        self.__robot=robot

    def load_image(self,image):
        image_loaded = self.__robot.call_service(self.name,'image_load',self.path+image+'.jpg')
        self.__robot.call_service(self.name,'image_paste',image_loaded.ir,0,0,False)      
