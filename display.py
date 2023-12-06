import tkinter as tk
import numpy as np
import simToMap as stm
from time import sleep
import DStarLite


RESOLUTION = 25
myArray = []
map = np.zeros((25, 25))
dstar = DStarLite.DStarLite((0,0), (RESOLUTION, RESOLUTION), map)
robotPos = (0,0)
goalPos= (RESOLUTION-1, RESOLUTION-1)


def pathToLine(path, canvasWidth=500):
    factor = (canvasWidth/RESOLUTION)
    return np.ndarray.flatten(np.rint(np.add(np.multiply(path, factor), RESOLUTION/2))).tolist()

def botPosToShape(botCord, canvasWidth=500):
    factor = (canvasWidth / RESOLUTION)
    spacing = factor*.1

    return (botCord[0]*factor+spacing, botCord[1]*factor+spacing,
            (botCord[0]+1)*factor-spacing, (botCord[1]+1)*factor-spacing)



def moveBot(newPos, delay=.25):
    global robotPos
    robotPos = newPos

    shape = botPosToShape(newPos)
    canvas.coords(robotOutline, shape)
    canvas.update()
    sleep(delay)
    canvas.coords(robotFill, shape)





def showGrid():
    global myArray
    npArray = np.array(myArray)
    if len(myArray) == 0:
        return
    map = stm.generate_map(npArray, 500, RESOLUTION)

    factor = 500/RESOLUTION
    for row in range(len(map)):
        for cell in range(len(map)):
            if map[row][cell]:
                canvas.create_rectangle(row*factor, cell*factor, (row+1)*factor, (cell+1)*factor,
                                        outline='blue', fill='', tags="box")



def circle(event):
    x = event.x
    y = event.y
    global myArray
    myArray.append((x, y))
    canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill="red", tags="point")

def runStep(delay=.2):
    global myArray
    npArray = np.array(myArray)
    if len(myArray) == 0:
        map = np.zeros((25,25))
    else: map = stm.generate_map(npArray, 500, RESOLUTION)

    path = DStarLite.dStarLitePath(map.tolist(), robotPos, goalPos)
    if (path == -1):
        canvas.coords(pathLine, 0,0,0,0)
        return 0

    if (len(path)<=2): # if at goal
        canvas.coords(pathLine, 0, 0, 0, 0)
        moveBot(goalPos, delay)
        return 0

    canvas.coords(pathLine, pathToLine(path))
    canvas.update()
    sleep(delay)
    moveBot(path[-2], delay)
    return 1

def autoRun():
    while runStep(.1):
        pass

def clearPoints():
    global myArray
    global map
    myArray = []
    map = np.zeros((25, 25))

    objects = canvas.find_withtag("point")
    objects += canvas.find_withtag("box")
    for o in objects:
        canvas.delete(o)




window = tk.Tk()
window.title("Pathfinding Simulator")
canvas = tk.Canvas(window, width=500, height=500, background="white")


canvas.bind("<B1-Motion>", circle)
canvas.pack(side=tk.LEFT)
pathLine = canvas.create_line(0,0,0,0)


# create goal
canvas.create_rectangle(botPosToShape(goalPos), fill='yellow')

robotFill = canvas.create_rectangle(botPosToShape(robotPos), fill='green', outline="")
robotOutline = canvas.create_rectangle(botPosToShape(robotPos), fill='', outline="black")


buttonFrame = tk.Frame()
stepButton = tk.Button(buttonFrame, text="Show Gridworld", command=showGrid)
stepButton.pack(fill="both", expand=True)

pathbutton = tk.Button(buttonFrame, text = "Run Step", command=runStep)
pathbutton.pack(fill="both", expand=True)

autoRunButton = tk.Button(buttonFrame, text="Auto Complete", command=autoRun)
autoRunButton.pack(fill="both", expand=True)

clearPointsButton = tk.Button(buttonFrame, text="Clear Points", command=clearPoints)
clearPointsButton.pack(fill="both", expand=True)

buttonFrame.pack(fill="both", expand=True, side=tk.RIGHT)
window.mainloop()

