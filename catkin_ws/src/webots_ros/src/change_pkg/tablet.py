#!/usr/bin/env python

class Tablet:
    def __init__(self, robot):
        self.display = Display(robot)
        self.speaker = Speaker(robot)


class Speaker:
    def __init__(self,robot,name='speaker',path='../../../../../Media/Audio/'):
        self.name=name
        self.path=path
        self.robot=robot

    def play_sound(self,sound):
        self.robot.call_service(self.name,'play_sound',self.path + sound + '.mp3', 1.0, 1.0, 0.0, False)

    def is_speaking(self):
        response = self.robot.call_service(self.name,'is_speaking')
        return response.value

    def speak(self,text,volume=1.0):
        while(self.is_speaking()):
            pass
        self.robot.call_service(self.name,'speak', text, volume)

    def speak_polyglot(self,it_IT=None,en_US=None,de_DE=None,es_ES=None,fr_FR=None,en_UK=None):
        for language, text in locals().items():
            if text is not None and language != 'self':
                self.robot.call_service(self.name, 'set_language', language.replace("_","-"))
                self.speak(text)                   

class Display:
    def __init__(self,robot,name='display',path='../../../../../Media/Image/'):              
        self.name=name
        self.path=path
        self.robot=robot

    def load_image(self,image):
        image_loaded = self.robot.call_service(self.name,'image_load',self.path+image+'.jpg')
        self.robot.call_service(self.name,'image_paste',image_loaded.ir,0,0,False)      
