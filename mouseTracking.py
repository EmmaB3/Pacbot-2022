import pyautogui
import math
from time import sleep
xCells = 26
yCells = 29
xMin = 26
yMin = 800
xMax = 675
yMax = 80
while(1):
    pos = pyautogui.position()
    print(pos)
    mouseX = pos[0]
    gridX = math.ceil((mouseX - xMin) * (xCells/(xMax-xMin)))
    mouseY = pos[1]
    gridY = math.ceil((mouseY - yMin) * (yCells/(yMax-yMin)))
    print("X: " + str(gridX) + ", Y: " + str(gridY))
    sleep(1) # 4 times per second
    #pyautogui.hotkey('ctrl', 'c')  
    #pyautogui.typewrite('clear\n', interval=0.1)  # useful for entering text, newline is Enter
