#!/usr/bin/env python
# -*- coding: utf-8 -*-
# generated by wxGlade 0.6.3 on Tue Sep 14 16:27:06 2010

import wx
import os
from wx.lib.statbmp import GenStaticBitmap


# begin wxGlade: extracode
# end wxGlade


ID_ABOUT=101
ID_OPEN=102
ID_SAVE=103
ID_CENTER=300
ID_EXIT=200


class MainFrame(wx.Frame):
    def __init__(self, *args, **kwds):
        # begin wxGlade: MainFrame.__init__
        kwds["style"] = wx.DEFAULT_FRAME_STYLE
        wx.Frame.__init__(self, *args, **kwds)
        self.map_panel = wx.ScrolledWindow(self, -1, style=wx.TAB_TRAVERSAL)
        
        # Menu Bar
        self.main_frame_menubar = wx.MenuBar()
        self.File = wx.Menu()
        self.Open_Map = wx.MenuItem(self.File, ID_OPEN, "&Open Map", "", wx.ITEM_NORMAL)
        self.File.AppendItem(self.Open_Map)
        self.Save_Map = wx.MenuItem(self.File, ID_SAVE, "&Save Map", "", wx.ITEM_NORMAL)
        self.File.AppendItem(self.Save_Map)
        self.Quit = wx.MenuItem(self.File, ID_EXIT, "&Quit", "", wx.ITEM_NORMAL)
        self.File.AppendItem(self.Quit)
        self.main_frame_menubar.Append(self.File, "&File")
        self.Help = wx.Menu()
        self.main_frame_menubar.Append(self.Help, "&Help")
        self.SetMenuBar(self.main_frame_menubar)
        # Menu Bar end
        self.main_frame_statusbar = self.CreateStatusBar(2, 0)
        
        # Tool Bar
        self.main_frame_toolbar = wx.ToolBar(self, -1, style=wx.TB_HORIZONTAL|wx.TB_DOCKABLE)
        self.SetToolBar(self.main_frame_toolbar)
        self.main_frame_toolbar.AddLabelTool(ID_OPEN, "Open Map", wx.Bitmap("Open_small.png", wx.BITMAP_TYPE_ANY), wx.NullBitmap, wx.ITEM_NORMAL, "Open Map", "Open Map")
        self.main_frame_toolbar.AddLabelTool(ID_SAVE, "Save Map", wx.Bitmap("save_small.png", wx.BITMAP_TYPE_ANY), wx.NullBitmap, wx.ITEM_NORMAL, "Save Map", "Save Map")
        self.main_frame_toolbar.AddSeparator()
        self.main_frame_toolbar.AddLabelTool(ID_CENTER, "Pick Center", wx.Bitmap("square.png", wx.BITMAP_TYPE_ANY), wx.NullBitmap, wx.ITEM_NORMAL, "Pick Map Center", "Pick on the location that should become the center of the map")
        # Tool Bar end
        self.static_bitmap = wx.StaticBitmap(self.map_panel, -1, wx.Bitmap("clear.png", wx.BITMAP_TYPE_ANY))
        #self.Bitmap = wx.Bitmap("/home/justin/Dropbox/Senior_Project/Map_Creator/clear.png", wx.BITMAP_TYPE_ANY)
        #self.static_bitmap = GenStaticBitmap(self, -1, self.Bitmap)


        self.__set_properties()
        self.__do_layout()

        self.Bind(wx.EVT_MENU, self.open_map, self.Open_Map)
        self.Bind(wx.EVT_MENU, self.save_map, self.Save_Map)
        self.Bind(wx.EVT_MENU, self.quit_app, self.Quit)
        self.Bind(wx.EVT_TOOL, self.open_map, id=ID_OPEN)
        self.Bind(wx.EVT_TOOL, self.save_map, id=ID_SAVE)
        self.Bind(wx.EVT_TOOL, self.pick_center, id=ID_CENTER)
        # end wxGlade
        self.dirname = '';
        #self.Bind(wx.EVT_LEFT_DOWN, self.OnLeftDown)
	#        self.main_frame_toolbar = wx.ToolBar(self, -1, style=wx.TB_VERTICAL|wx.TB_DOCKABLE)
	self.Maximize();
   

    def __set_properties(self):
        # begin wxGlade: MainFrame.__set_properties
        self.SetTitle("Map Builder")
        self.main_frame_statusbar.SetStatusWidths([500, 200])
        # statusbar fields
        main_frame_statusbar_fields = ["Open", "NA"]
        for i in range(len(main_frame_statusbar_fields)):
            self.main_frame_statusbar.SetStatusText(main_frame_statusbar_fields[i], i)
        self.main_frame_toolbar.SetToolBitmapSize((16, 16))
        self.main_frame_toolbar.Realize()
        self.map_panel.SetMinSize((1920,1200))
        self.map_panel.SetScrollRate(10, 10)
        # end wxGlade

    def __do_layout(self):
        # begin wxGlade: MainFrame.__do_layout
        sizer_1 = wx.BoxSizer(wx.VERTICAL)
        sizer_2 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_2.Add(self.static_bitmap, 0, 0, 0)
        self.map_panel.SetSizer(sizer_2)
        sizer_1.Add(self.map_panel, 1, wx.EXPAND|wx.FIXED_MINSIZE, 0)
        self.SetSizer(sizer_1)
        sizer_1.Fit(self)
        self.Layout()
        # end wxGlade

   

    def quit_app(self, event): # wxGlade: MainFrame.<event_handler>
      #print "Event handler `quit_app' not implemented"
      exit = wx.MessageDialog( self, " Are you sure you want to exit? \n",
                     "Closing", wx.YES_NO)
      result = exit.ShowModal()
      if result == wx.ID_YES:
         self.Close(True) 
      #event.Skip()
      return;


    def open_map(self, event): # wxGlade: MainFrame.<event_handler>
         #print "Event handler `open_map' not implemented"
         dlg = wx.FileDialog(self, "Choose an image to open", self.dirname, "", "*.*", wx.OPEN)
         if dlg.ShowModal() == wx.ID_OK:
            self.filename=dlg.GetFilename()
            self.dirname=dlg.GetDirectory()
            #self.map = wx.StaticBitmap(self.map_panel, -1, wx.Bitmap(self.dirname +"/" +self.filename, wx.BITMAP_TYPE_ANY))
            dc = wx.PaintDC(self.static_bitmap)
            dc.Clear()
            self.file = self.dirname +"/" +self.filename
            self.Bitmap =  wx.Bitmap(self.file, wx.BITMAP_TYPE_ANY)
            self.width = self.Bitmap.GetWidth();
            self.height = self.Bitmap.GetHeight();
            self.true_x = self.width/2
            self.true_y = self.height/2
            s = "Center:\t" +str(self.true_x) + ", " + str(self.true_y)
            self.main_frame_statusbar.SetStatusText( s, 1)
            print "Image Height: " + str(self.height)
            print "Image Width: " + str(self.width)
            
            #self.map_panel.SetSize(self.Bitmap.GetSize())
            #dc.DrawBitmap( self.Bitmap, 0, 0, True)
            self.static_bitmap.SetBitmap(self.Bitmap)

        #event.Skip()
         dlg.Destroy();
         return

    def save_map(self, event): # wxGlade: MainFrame.<event_handler>
        #print "Event handler `save_map' not implemented"
        dlg = wx.FileDialog(self, "Choose where to save the Simulator Map", "../Simulator/", "", "*.*", wx.SAVE)
        if dlg.ShowModal() == wx.ID_OK:
            self.save_filename=dlg.GetFilename()
            self.save_dirname=dlg.GetDirectory()
            self.save_file = self.save_dirname +"/"+self.save_filename + ".ym"
            print self.save_file
            #command = "./makeGridMapUtil " + self.file + " " + self.save_file  + " " + self.true_x + " " + self.true_y
            #print command
            os.system("./makeGridMapUtil " + self.file + " " + self.save_file  + " " + str(self.true_x) + " " + str(self.true_y))
        #event.Skip()
        dlg.Destroy()
        
        dlg = wx.MessageDialog(self, 'Simulation map file has been saved at ' + self.save_file, 'File Saved', wx.OK|wx.ICON_INFORMATION)
        dlg.ShowModal()
        dlg.Destroy()
        return

    def pick_center(self, event): # wxGlade: MainFrame.<event_handler>
        #print "Event handler `pick_center' not implemented"
        self.static_bitmap.Bind(wx.EVT_LEFT_DOWN, self.OnLeftDown)
        #self.Bind(wx.EVT_LEFT_DOWN, self.OnLeftDown)
        #wx.EVT_LEFT_DOWN(self.map_panel, self.OnLeftClick)
        #event.Skip()
        return

    def OnLeftDown(self, event):
        #print "Left Button Pressed"
        x, y = event.GetPosition()
        s = "Center:\t" +str(x) + ", " + str(y)
        print s
        self.clicked_point = wx.Point(x,y)
        shift = self.map_panel.CalcUnscrolledPosition(self.clicked_point)
        print shift
        true_x, true_y = shift.Get()
        s = "Center:\t" +str(true_x) + ", " + str(true_y)
        self.main_frame_statusbar.SetStatusText( s, 1)
        #self.true_x = self.width -true_x
        self.true_x = true_x
        self.true_y = self.height -true_y
        self.static_bitmap.Unbind(wx.EVT_LEFT_DOWN)
        
        return

# end of class MainFrame


if __name__ == "__main__":
    app = wx.PySimpleApp(0)
    wx.InitAllImageHandlers()
    main_frame = MainFrame(None, -1, "")
    app.SetTopWindow(main_frame)
    main_frame.Show()
    app.MainLoop()
