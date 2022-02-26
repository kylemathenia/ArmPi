__target_color = ('red', 'green', 'blue')
x=6
def setTargetColor(target_color):
    global __target_color

    #print("COLOR", target_color)
    __target_color = target_color

print(__target_color)
setTargetColor('red')
print(__target_color)