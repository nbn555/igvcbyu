import Tkinter
import test
import os
import camera
import threading

class ShowCamera(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		threading.Thread.daemon = True
		
		self.param = None
		
	def update(self,param):
		self.param = param
		
	def run(self):
		if self.param != None:
			boxes = -1
			if self.param["display_boxes"] == True:
				boxes = 1
			else:
				boxes = 0
			
			camera.update(int(self.param["grid_size"]),
						  int(self.param["threshold"]),
						  float(self.param["percent_filled"]),
						  int(self.param["erode_amount"]),
						  int(self.param["hsv_vector"]),
						  int(boxes))
	

class CameraParamUI(Tkinter.Tk):
	def __init__(self,parent, camera):
		Tkinter.Tk.__init__(self,parent)
		
		self.parent = parent
		self.camera = camera
		self.fileName = "YClops.ini"
		
		self.initialize()
		
	def initialize(self):
		param = self.loadFile(self.fileName)
	
		self.grid()
		
		"""
		UI Needed:
			grid_size - scrollbar
			threshold - slider
			percent filled - slider
			erode_amount - spinbox
			hsv_vector - ?
			display boxes - checkbox
		
		""" 
		
		""" HSV Vector """
		self.hsvVector = Tkinter.Listbox(self, width = 4, height = 3)
		self.hsvVector.insert(1, "0")
		self.hsvVector.insert(2, "1")
		self.hsvVector.insert(3, "2")
		
		self.hsvVector.grid(column=0, row=4)
		
		""" Grid Size """
		labelText = Tkinter.StringVar()
		labelText.set("Grid Size: ")
		label = Tkinter.Label(self,textvariable=labelText)
		label.grid(column=4,row=0)
		
		self.gridSize = Tkinter.Spinbox(self, from_=1, to=100)
		self.gridSize.grid(column=5, row=0)
		self.gridSize.delete(0,"end")
				
		
		""" Erode Amount """
		labelText = Tkinter.StringVar()
		labelText.set("Erode Amount: ")
		label = Tkinter.Label(self,textvariable=labelText)
		label.grid(column=2,row=0)
		
		self.erodeAmount = Tkinter.Spinbox(self, from_=0, to=10)
		self.erodeAmount.grid(column=3, row=0)
		self.erodeAmount.delete(0,"end")
		
		""" Display Boxes """
		self.displayBoxes = Tkinter.BooleanVar()
		self.displayBoxesCheck = Tkinter.Checkbutton(self,
								 	variable = self.displayBoxes,
								 	text = "Display Boxes")
		self.displayBoxesCheck.grid(column=1,row=0)
		
		""" Distort """
		self.distort = Tkinter.BooleanVar()
		self.distortCheck = Tkinter.Checkbutton(self,
								 	variable = self.distort,
								 	text = "Distort")
		self.distortCheck.grid(column=2,row=0)
		
		""" THRESHOLD SLIDER"""
		self.threshold = Tkinter.IntVar()
		self.thresholdSlider = Tkinter.Scale(self, variable = self.threshold,
									orient=Tkinter.HORIZONTAL, length=500,
									tickinterval=50, from_=0, to=255,
									label="Threshold", sliderlength=20,
									repeatdelay=1)
		self.thresholdSlider.grid(column=1,row=1,columnspan=3, sticky='EW')
		
		
		"""PERCENT FILLED"""
		self.percentFilled = Tkinter.DoubleVar()
		
		self.percentFilledSlider = Tkinter.Scale(self, variable = self.percentFilled,
									orient=Tkinter.HORIZONTAL, length=500,
									tickinterval=0.10, from_=0.0, to=1.0,
									label="Percent Fill", sliderlength=20,
									repeatdelay=1, resolution=.01)
		self.percentFilledSlider.grid(column=1,row=2,columnspan=3, sticky='EW')
		
		
		""" SAVE BUTTON """
		saveButton = Tkinter.Button(self, text="Save", command=self.OnSaveButtonClick)
		saveButton.grid(column=2,row=3)


		""" UPDATE BUTTON """
		updateButton = Tkinter.Button(self, text="Update", command=self.OnUpdateButtonClick)
		updateButton.grid(column=1,row=3)
		
		""" SETTING INITIAL VALUES"""
		self.hsvVector.select_set(param["hsv_vector"])
		self.percentFilledSlider.set(param["percent_filled"])
		self.thresholdSlider.set(param["threshold"])
		self.gridSize.insert(0,param["grid_size"])
		self.erodeAmount.insert(0,param["erode_amount"])
		self.distortCheck.select()
		self.displayBoxesCheck.deselect()
		
    

	def OnUpdateButtonClick(self):
		
		param = self.getUIValues()

		camera.update(int(param["grid_size"]),
					  int(param["threshold"]),
					  float(param["percent_filled"]),
					  int(param["erode_amount"]),
					  int(param["hsv_vector"]),
					  int(param["display_boxes"]),
					  int(param["distort"]))
		
                

	def OnSaveButtonClick(self):
		param = self.getUIValues()
		
		self.saveFile(self.fileName, param)
		
	def getUIValues(self):
		print "threshold: " + str(self.threshold.get())
		print "percent filled: " + str(self.percentFilled.get())
		print "display boxes: " + str(self.displayBoxes.get())
		print "erode amount: " + str(self.erodeAmount.get())
		print "grid size: " + str(self.gridSize.get())
		print "HSV vector: " + str(self.hsvVector.curselection())
		
		param = {"grid_size":self.gridSize.get(),
				 "percent_filled":self.percentFilled.get(),
		         "threshold":self.threshold.get(),
		         "erode_amount":self.erodeAmount.get(),
		         "hsv_vector":int(self.hsvVector.curselection()[0]),
		         "display_boxes":self.displayBoxes.get(),
		         "distort":self.distort.get()}
		        
		return param;
		
	def saveFile(self, fileName, param):
		newFileName = fileName + "~"
		os.rename( fileName, newFileName )
		
		src = open(newFileName, 'r')
		dst = open(fileName, 'w')
			
		for line in src.readlines():
			if line.find("grid_size") != -1:
				dst.write("grid_size = " + str(param["grid_size"]) + "\n")
				
			elif line.find("percent_filled") != -1:
				dst.write("percent_filled = " + str(param["percent_filled"]) + "\n")
								
			elif line.find("threshold") != -1:
				dst.write("threshold = " + str(param["threshold"]) + "\n")
								
			elif line.find("erode_amount") != -1:
				dst.write("erode_amount = " + str(param["erode_amount"]) + "\n")
								
			elif line.find("hsv_vector") != -1:
				dst.write("hsv_vector = " + str(param["hsv_vector"]) + "\n")
			
			else:
				dst.write(line)
		
				
	def loadFile(self, fileName):
		with open(fileName) as f:
		
			for line in f.readlines():
				if line.find("grid_size") != -1:
					temp, grid = line.split("=")
					
				elif line.find("percent_filled") != -1:
					temp, percent = line.split("=")
									
				elif line.find("threshold") != -1:
					temp, threshold = line.split("=")
									
				elif line.find("erode_amount") != -1:
					temp, erode = line.split("=")
									
				elif line.find("hsv_vector") != -1:
					temp, hsv = line.split("=")
				
								
		return {"grid_size":int(grid), "percent_filled":float(percent),
		        "threshold":int(threshold), "erode_amount":int(erode),
		        "hsv_vector":int(hsv)}
				
				
				
	
	
if __name__ == "__main__":
	# Create camera module here
	#c = ShowCamera()
	#c.start()

	print "Starting user interface..."
	app = CameraParamUI(None, None)
	app.title('Camera Parameters')
	app.mainloop()