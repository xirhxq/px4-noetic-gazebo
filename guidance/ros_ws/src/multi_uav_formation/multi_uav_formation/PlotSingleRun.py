import os
import glob
import pickle
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from datetime import timedelta
from pathlib import Path


class PlotSingleRun:
    def __init__(self, useLastRun=False):
        self.folderName = self.selectRunDirectory(useLastRun)
        self.dataFiles = glob.glob(os.path.join(self.folderName, 'data_*.pkl'))
        self.runData = [pickle.load(open(file, 'rb')) for file in self.dataFiles]

        self.timeStep = self.runData[0]['params']['execution']['time_step']
        self.leadersId = self.runData[0]['config']['leaders']
        self.headVehicleId = self.runData[0]['config']['head']

        self.synchronizeDataStartTime()
        self.absolutePositions = self.extractAbsolutePositions()
        self.relativePositions = self.calculateRelativePositions()

        self.show3DFigures = False

    def selectRunDirectory(self, useLastRun):
        roots = Path(__file__).parents
        runFiles = sorted(glob.glob(os.path.join(roots[3], 'data/multi_uav_formation/SingleRun/*')))
        if useLastRun:
            return runFiles[-1]
        for idx, file in enumerate(runFiles):
            print(f'[{idx}]: {file}')
        selectedIndex = int(input('Choose file: '))
        return runFiles[selectedIndex]

    def synchronizeDataStartTime(self):
        maxStartTime = max(run['data'][0]['systemTime'] for run in self.runData)
        for run in self.runData:
            run['data'] = [entry for entry in run['data'] if entry['systemTime'] >= maxStartTime]

    def extractAbsolutePositions(self):
        return {
            run['selfnumber']: [[entry['systemTime'], entry['mePositionENU']] for entry in run['data']]
            for run in self.runData
        }

    def calculateRelativePositions(self):
        relativePositions = {self.headVehicleId: [[time, pos - pos] for time, pos in self.absolutePositions[self.headVehicleId]]}
        for vehicleId, data in self.absolutePositions.items():
            if vehicleId == self.headVehicleId:
                continue
            relativePositions[vehicleId] = self.computeRelativePosition(vehicleId, data)
        return relativePositions

    def computeRelativePosition(self, vehicleId, data):
        relativeData = []
        headData = self.absolutePositions[self.headVehicleId]
        headIndex = 0
        for time, position in data:
            while headData[headIndex][0] < time and headIndex < len(headData) - 1:
                headIndex += 1
            headPosition = self.interpolate(headData[headIndex - 1], headData[headIndex], time)
            relativeData.append([time, position - headPosition])
        return relativeData

    def interpolate(self, prevData, nextData, targetTime):
        prevTime, prevValue = prevData
        nextTime, nextValue = nextData
        return prevValue * (nextTime - targetTime) / (nextTime - prevTime) + nextValue * (targetTime - prevTime) / (nextTime - prevTime)

    def getTimeRange(self):
        allTimes = [time for data in self.absolutePositions.values() for time, _ in data]
        return min(allTimes), max(allTimes)

    def initializePlot(self, ax, relative=False):
        dataSource = self.relativePositions if relative else self.absolutePositions
        allPositions = [np.array([pos for _, pos in data]) for data in dataSource.values()]

        minPoint = np.min([pos.min(axis=0) for pos in allPositions], axis=0)
        maxPoint = np.max([pos.max(axis=0) for pos in allPositions], axis=0)


        ax.set_xlim(minPoint[0], maxPoint[0])
        ax.set_ylim(minPoint[1], maxPoint[1])
        ax.set_zlim(minPoint[2], maxPoint[2])

    def createAnimation(self, relative=False, speed=1):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        dataSource = self.relativePositions if relative else self.absolutePositions
        startTime, endTime = self.getTimeRange()

        animationLength = (endTime - startTime).total_seconds()

        frames = int((animationLength / self.timeStep) / speed)

        def updateFrame(frame):
            ax.clear()
            self.initializePlot(ax, relative)

            for vehicleId, data in dataSource.items():
                times, positions = zip(*data)
                currentTime = startTime + timedelta(seconds=frame * self.timeStep * speed)
                currentIndex = np.searchsorted(times, currentTime)

                if currentIndex >= len(positions):
                    currentIndex = len(positions) - 1

                settings = dict(
                    color='red' if vehicleId <= self.leadersId else 'blue',
                    marker='o',
                    s=10,
                    alpha=0.5 if vehicleId <= self.leadersId else 0.2
                )

                ax.scatter(*np.array(positions[currentIndex]).T, **settings)

            ax.set_xlabel('x / m')
            ax.set_ylabel('y / m')
            ax.set_zlabel('z / m')
            ax.set_title('Relative Formation' if relative else 'Absolute Formation')

        anim = animation.FuncAnimation(fig, updateFrame, frames=frames, interval=100, blit=False)
        outputFileName = os.path.join(self.folderName, 'relative.gif' if relative else 'absolute.gif')
        anim.save(outputFileName, fps=int(speed / self.timeStep))

        if self.show3DFigures:
            plt.show()

    def createPlot(self, relative=False):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        startTime, endTime = self.getTimeRange()
        norm = plt.Normalize(0, (endTime - startTime).total_seconds())
        plt.colorbar(plt.cm.ScalarMappable(norm=norm), ax=ax, label='Time (s)')

        self.initializePlot(ax, relative)
        dataSource = self.relativePositions if relative else self.absolutePositions

        for vehicleId, data in dataSource.items():
            positions = np.array([pos for _, pos in data])
            settings = dict(
                c='red' if vehicleId <= self.leadersId else 'blue',
                s=10,
                alpha=0.5 if vehicleId <= self.leadersId else 0.2
            )
            ax.scatter(*positions.T, **settings)

        ax.set_xlabel('x / m')
        ax.set_ylabel('y / m')
        ax.set_zlabel('z / m')
        ax.set_title('Relative Formation' if relative else 'Absolute Formation')

        pictureFileName = os.path.join(self.folderName, 'rela.png' if relative else 'abso.png')
        fig.savefig(pictureFileName)
        print(f'Picture saved to {pictureFileName}')

        if self.show3DFigures:
            plt.show()

    def plotVelGradientAndAcceleration(self, windowSize = 3):

        for vehicleData in self.runData:
            meAcc = np.array([entry['meAccelerationENU'] for entry in vehicleData['data']])
            meVel = np.array([entry['meVelocity'] for entry in vehicleData['data']])
            time = np.array([entry['t'] for entry in vehicleData['data']])


            N = len(time)
            meVelGrad = np.zeros_like(meVel)

            for i in range(windowSize, N - windowSize):
                delta_t = time[i + windowSize] - time[i - windowSize]
                if delta_t > 0:
                    meVelGrad[i] = (meVel[i + windowSize] - meVel[i - windowSize]) / delta_t


            fig, axs = plt.subplots(3, 1)

            for i in range(3):
                axs[i].plot(meVelGrad[:, i])
                axs[i].plot(meAcc[:, i])
                axs[i].set_title(f'Velocity Gradient in NED {["X", "Y", "Z"][i]}')
                axs[i].set_xlabel('Time')
                axs[i].set_ylabel('Velocity Gradient and Acceleration')

            fig.suptitle(f'Velocity Gradient in NED for {vehicleData["selfnumber"]}')
            plt.savefig(os.path.join(self.folderName, f'{vehicleData["selfnumber"]}_vel_grad.png'))

            if self.show3DFigures:
                plt.show()

    def plotAcceResponse(self):

        for vehicleData in self.runData:

            plt.figure()

            acceResponse = np.array([entry['meAccelerationENU'] for entry in vehicleData['data']])
            acceENU = np.array([entry['u'] for entry in vehicleData['data']])
            time = np.array([entry['t'] for entry in vehicleData['data']])

            if len(acceENU.shape) < 2:
                continue

            plt.subplot(3, 1, 1)
            plt.plot(time, acceENU[:, 0], 'r', linewidth=2, label="Acceleration command E")
            plt.plot(time, acceResponse[:, 0], '--', color=[1, 0, 0, 0.5], label="Acceleration response E")
            # plt.ylim(-20,20)
            plt.legend()

            plt.subplot(3, 1, 2)
            plt.plot(time, acceENU[:, 1], 'g', linewidth=2, label="Acceleration command N")
            plt.plot(time, acceResponse[:, 1], '--', color=[0, 1, 0, 0.5], label="Acceleration response N")
            # plt.ylim(-20,20)
            plt.legend()

            plt.subplot(3, 1, 3)
            plt.plot(time, acceENU[:, 2], 'b', linewidth=2, label="Acceleration command U")
            plt.plot(time, acceResponse[:, 2], '--', color=[0, 0, 1, 0.5], label="Acceleration response U")
            plt.xlabel('Time (s)')
            # plt.ylim(-20,20)
            plt.legend()
            
            plt.savefig(os.path.join(self.folderName, f'{vehicleData["selfnumber"]}_AccelerationResponse.png'))
            plt.close()

    def createPlot2D(self):

        fig, ax = plt.subplots()  

        startTime, endTime = self.getTimeRange()
        norm = plt.Normalize(0, (endTime - startTime).total_seconds())
        fig.colorbar(plt.cm.ScalarMappable(cmap='viridis', norm=norm), ax=ax, label='time / s')

        dataSource = self.absolutePositions

        for vehicleId, data in dataSource.items():
            positions = np.array([pos for _, pos in data])
            times = np.array([(time - startTime).total_seconds() for time, _ in data])
            if vehicleId <= self.leadersId:
                ax.scatter(positions[:, 0], positions[:, 1], s=1, c='r')  
            else:
                ax.scatter(
                    positions[:, 0], positions[:, 1], c=times,
                    cmap='viridis', s=1,
                )

        ax.set_xlabel('x / m')
        ax.set_ylabel('y / m')
        ax.set_title('Absolute 2D Formation')

        for i in range(self.runData[-1]['sceneConfig']['nObstacle']):
            center = self.runData[-1]['sceneConfig']['obstacleData'][str(i)]['centerENU']
            radius = self.runData[-1]['sceneConfig']['obstacleData'][str(i)]['radius']
            circle = plt.Circle(center, radius, edgecolor='b', facecolor='none')
            ax.add_patch(circle)

        pictureFileName = os.path.join(self.folderName, 'abso2D.png')
        fig.savefig(pictureFileName)
        
        if self.show3DFigures:
                plt.show()
        plt.close(fig)
        print(f'Picture saved to {pictureFileName}')

    def askShow(self):
        while True:
            op = input('Show picture? (y/n) ')
            if op.lower() == 'y':
                self.show3DFigures = True
                return
            elif op.lower() == 'n':
                self.show3DFigures = False
                return
            else:
                print('Invalid input, please try again...')

    def drawAll(self):
        self.askShow()
        self.createPlot()
        self.createPlot(relative=True)
        self.createAnimation(speed=30)
        self.createAnimation(relative=True, speed=30)
        self.plotVelGradientAndAcceleration()
        self.plotAcceResponse()
        self.createPlot2D()


if __name__ == "__main__":
    plotter = PlotSingleRun()
    plotter.drawAll()
