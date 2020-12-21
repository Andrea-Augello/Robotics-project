#!/usr/bin/env python
robot = None

class Tablet:
    def __init__(self, r):
        self.display = Display()
        self.speaker = Speaker()
        global robot
        robot = r

class Speaker:
    def __init__(self,name='speaker',path='../../../../../Media/Audio/'):
        self.name=name
        self.path=path

    def play_sound(self,sound):
        robot.call_service(self.name,'play_sound',self.path + sound + '.mp3', 1.0, 1.0, 0.0, False)

    def is_speaking(self):
        response = robot.call_service(self.name,'is_speaking')
        return response.value

    def speak(self,text,volume=1.0):
        while(self.is_speaking()):
            pass
        robot.call_service(self.name,'speak', text, volume)

    def speak_polyglot(self,it_IT=None,en_US=None,de_DE=None,es_ES=None,fr_FR=None,en_UK=None):
        for language, text in locals().items():
            if text is not None and language != 'self':
                robot.call_service(self.name, 'set_language', language.replace("_","-"))
                self.speak(text)                   

class Display:
    def __init__(self,name='display',path='../../../../../Media/Image/'):              
        self.name=name
        self.path=path

    def load_image(self,image):
        image_loaded = robot.call_service(self.name,'image_load',self.path+image+'.jpg')
        robot.call_service(self.name,'image_paste',image_loaded.ir,0,0,False)      