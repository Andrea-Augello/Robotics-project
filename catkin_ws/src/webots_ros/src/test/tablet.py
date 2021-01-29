#!/usr/bin/env python3
import change_pkg.utils as utils

class Tablet:
    def __init__(self):
        self.display = Display()
        self.speaker = Speaker()

    def greetings(self):
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
    def __init__(self,name='speaker'):
        self.name=name

    
    def speak_polyglot(self,it_IT=None,en_US=None,de_DE=None,es_ES=None,fr_FR=None,en_UK=None):
        message=''
        for language, text in locals().items():
            if text is not None and language != 'self' and language != 'message':
                message+=language.replace("_","-")+"@"+text+"|"
        message=message[:-1]
        utils.publish_interaction(self.name, message)
        


class Display:
    def __init__(self,name='display'):              
        self.name=name

    def load_image(self,image):
        utils.publish_interaction(self.name, image)
