# image testing
import time
import Tkinter as tk
import tkFont
import Fusion.Gui.GuiTypes as gt
import Fusion.Utils.IVTimer as IVTimer
#import Fusion.Gui.GuiUtils as gut

class ActiveImageWidget(tk.Canvas):
  """ Active Image Widget Class """

  #--
  def __init__(self, master, cnf={}, filenames=[], fps=0.125, **kw):
    """ Initialize the Active Image widget The display of the images in the
        specified list cycle at the given frames/second rate.

        Parameters:
          master    - master widget
          cnf       - Tkinter.Text widget standard cnf values
          filenames - list of image file path names
          fps       - frames/second cycle rate
          **kw      - Tkinter.Text widget standard keyword argments
    """
    self.mIndex     = 0
    self.mFps       = fps
    self.mImgList   = []
    self.mImgWidth  = 1
    self.mImgHeight = 1

    # load and build the image
    for filename in filenames:
      img = tk.PhotoImage(file=filename)
      width = img.width()
      height = img.height()
      self.mImgList += [img]
      if width > self.mImgWidth:
        self.mImgWidth = width
      if height > self.mImgHeight:
        self.mImgHeight = height

    self.mImgWidth += 2
    self.mImgHeight += 2

    self.mNumImages = len(self.mImgList)

    # set to a fixed size font so the dimension arthimetic works
    #font = tkFont.Font(master, font=gt.FontCour10Bold)
    #fontWidth  = font.measure('M')
    #fontHeight = font.metrics()['linespace']

    # text widget to show the image
    tk.Canvas.__init__(self, master, cnf=cnf, 
                relief=tk.FLAT, borderwidth=0,
                height=self.mImgHeight, width=self.mImgWidth,
                **kw)

    self.mCenter = (self.mImgWidth/2, self.mImgHeight/2)

    self._mIterator = None
    self.mId = self.after_idle(self._CbIterStart)

  #--
  def destroy(self):
    """ Destroy this widget. """
    print('rdk', 'destroy')
    if self._mIterator:
      self._mIterator.cancel()
    tk.Canvas.destroy(self)

  def _CbIterStart(self):
    """ Start image sequencer iterator. """
    print('rdk', 'after_idle')
    self.after_cancel(self.mId)
    if self.mNumImages > 1:
      self._mIterator = IVTimer.IVTimer(self.mFps, self.mFps, self._CbIterNext)
      self._mIterator.start()
      self.mId = -1
    else:
      self.mId = self.create_image(self.mCenter, anchor=tk.CENTER,
                                    image=self.mImgList[self.mIndex])

  #--
  def _CbIterNext(self, ivt):
    """ Image sequencing iterator. """
    # mainloop may not have beed created or is in process of being destroyed
    try:
      print('rdk', 'index=', self.mIndex)
      #self.itemconfigure(self.mId, image=self.mImgList[self.mIndex])
      if self.mId >= 0:
        self.delete(self.mId)
      self.mId = self.create_image(self.mCenter, anchor=tk.CENTER,
                                    image=self.mImgList[self.mIndex])
      self.mIndex = (self.mIndex + 1) % self.mNumImages
    except RuntimeError:
      pass


root = tk.Tk()
imgName0 = '/prj/src/fusion/Fusion-1.0/Fusion/Gui/Images/BlockyWalk0.gif'
imgName1 = '/prj/src/fusion/Fusion-1.0/Fusion/Gui/Images/BlockyWalk1.gif'
imgName2 = '/prj/src/fusion/Fusion-1.0/Fusion/Gui/Images/BlockyWalk2.gif'
imgName3 = '/prj/src/fusion/Fusion-1.0/Fusion/Gui/Images/BlockyWalk3.gif'
#w = ActiveImageWidget(root, filenames=[imgName0], fps=0.25) 
w = ActiveImageWidget(root, filenames=[imgName0, imgName1, imgName2, imgName3],
    fps=0.25) 
w.grid(row=0, column=0)
time.sleep(2)
root.mainloop()
