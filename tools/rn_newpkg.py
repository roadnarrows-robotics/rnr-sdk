#!/usr/bin/python
################################################################################
#
# Package:  tools
#
# File:     pkg_gen.py
#
# Version:
#   $LastChangedDate: 2013-05-09 11:38:00 -0600 (Thu, 09 May 2013) $
#   $Rev: 2942 $
#
# Description:
#   RN package generation utility
#
#   Create a brand spanking new empty package.
#
#   TBD
#
# Author: Daniel Packard (daniel@roadnarrows.com)
#
# Copyright (C) 2013.  RoadNarrows LLC.
# All Rights Reserved
#
# Permission is hereby granted, without written agreement and without
# license or royalty fees, to use, copy, modify, and distribute this
# software and its documentation for any purpose, provided that
# (1) The above copyright notice and the following two paragraphs
# appear in all copies of the source code and (2) redistributions
# including binaries reproduces these notices in the supporting
# documentation.   Substantial modifications to this software may be
# copyrighted by their authors and need not follow the licensing terms
# described here, provided that the new terms are clearly indicated in
# all files where they apply.
#
# IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
# OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
# PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
# DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
# EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.
#
# THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
# "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
# PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
#
################################################################################

import Tkinter
import os.path
from datetime import datetime as date
import re

import newpkg_utils as utils

class CompoundEntry:
  def __init__(self, parent, label, width=20):
    self.frame = Tkinter.Frame(
        parent)
    # label
    self.label = Tkinter.Label(
        self.frame,
        text=label,
        width=14,
        anchor="e"
      )

    # entry box
    self.entry = Tkinter.Entry(
        self.frame,
        width=width)
  
  def pack(self, side="top"):
    self.frame.pack(fill=Tkinter.X, side=side)
    self.label.pack(padx=4, pady=4, side="left")
    self.entry.pack(padx=0, pady=0, side="left")

  def get(self):
    return self.entry.get()

  def set(self, val):
    self.entry.delete(0, Tkinter.END)
    return self.entry.insert(0, val)

class CompoundCheckbutton:
  def __init__(self, parent, label):
    self.val=Tkinter.IntVar()
    self.frame = Tkinter.Frame(
        parent)
    # label
    self.label = Tkinter.Label(
        self.frame,
        text=label,
        width=8,
        anchor="e"
      )

    # check box
    self.checkbutton = Tkinter.Checkbutton(
        self.frame,
        variable=self.val
      )
  
  def pack(self, side="top"):
    self.frame.pack(fill=Tkinter.X, side=side)
    self.label.pack(padx=4, pady=4, side="left")
    self.checkbutton.pack(padx=0, pady=0, side="left")

  def get(self):
    return self.val.get()

  def select(self):
    return self.checkbutton.select()

class HRule:
  def __init__(self, parent):
    self.rule=Tkinter.Frame(
        parent,
        relief=Tkinter.RAISED, 
        borderwidth=1,
        height=6)

  def pack(self, side="top"):
    self.rule.pack(fill=Tkinter.X, expand=0, side=side, pady=3)

class PkgGenDialog:
  def __init__(self):
    self.initGui()

  def initGui(self):
    self.main_frame = Tkinter.Frame(relief=Tkinter.RAISED, borderwidth=1)
    self.main_frame.pack(fill=Tkinter.BOTH, expand=1, side="top", padx=8)

    self.pkgdata_frame= Tkinter.Frame(self.main_frame)
    self.pkgdata_frame.pack(side="top",fill=Tkinter.X)

  # package data
    self.pkgname_entry = CompoundEntry(self.pkgdata_frame, "Package Name: ")
    self.pkgname_entry.pack()

    # package owner info
    self.author_frame = Tkinter.Frame(self.main_frame)
    self.author_frame.pack(side="top",fill=Tkinter.X)

    self.owner_entry = CompoundEntry(self.author_frame, "Package Owner: ")
    self.owner_entry.set("RoadNarrows LLC")
    self.owner_entry.pack(side="left")

    self.website_entry  = CompoundEntry(self.author_frame, "Website: ")
    self.website_entry.set("http://roadnarrows.com")
    self.website_entry.pack(side="left")

    # author info
    self.author_frame = Tkinter.Frame(self.main_frame)
    self.author_frame.pack(side="top",fill=Tkinter.X)

    self.author_entry = CompoundEntry(self.author_frame, "Primary Author: ")
    self.author_entry.pack(side="left")

    self.email_entry  = CompoundEntry(self.author_frame, "Email: ")
    self.email_entry.pack(side="left")

    self.desc_entry = CompoundEntry(self.main_frame, "Description: ", width=60)
    self.desc_entry.pack(side="top")

    # license radio buttons
    self.license_frame = Tkinter.Frame(self.main_frame)
    self.license_frame.pack(side="top",fill=Tkinter.X)

    self.features_label = Tkinter.Label(
        self.license_frame,
        text="License: "
      )
    self.features_label.pack(side="left")
    
    self.license = Tkinter.StringVar()
    b1 = Tkinter.Radiobutton(self.license_frame,
                        text="GPL",
                        variable=self.license, 
                        value="GPL")
    b1.pack(side="left")
    b2 = Tkinter.Radiobutton(self.license_frame,
                        text="LGPL",
                        variable=self.license, 
                        value="LGPL",
                        )
    b2.pack(side="left")
    b2.select()
    b3 = Tkinter.Radiobutton(self.license_frame,
                        text="BSD",
                        variable=self.license, 
                        value="BSD")
    b3.pack(side="left")

    self.rule1 = HRule(self.main_frame)
    self.rule1.pack()

    print "license: " + self.license.get()
  # additional project options
    self.features_frame = Tkinter.Frame(self.main_frame)
    self.features_frame.pack(fill=Tkinter.X, side="top")

    self.features_label = Tkinter.Label(
        self.features_frame,
        text="Project Features: "
      )
    self.features_label.pack(anchor="w")
    
    self.sw_cb = CompoundCheckbutton(self.features_frame, "SW: ")
    self.sw_cb.select()
    self.sw_cb.pack(side="left")

    self.hw_cb = CompoundCheckbutton(self.features_frame, "HW: ")
    self.hw_cb.select()
    self.hw_cb.pack(side="left")

    self.fw_cb = CompoundCheckbutton(self.features_frame, "FW: ")
    self.fw_cb.pack(side="left")

    self.dpkg_cb = CompoundCheckbutton(self.features_frame, "DPKG: ")
    self.dpkg_cb.select()
    self.dpkg_cb.pack(side="left")

    self.rule2 = HRule(self.main_frame)
    self.rule2.pack()

  # embed 3rd party libs (tinyxml, gtest, etc)
    self.embeddedlibs_frame = Tkinter.Frame(self.main_frame)
    self.embeddedlibs_frame.pack(fill=Tkinter.X, side="top")

    self.features_label = Tkinter.Label(
        self.embeddedlibs_frame,
        text="Embed 3rd party libraries in the project: "
      )
    self.features_label.pack(anchor="w")
    
    self.gtest_cb = CompoundCheckbutton(self.embeddedlibs_frame, "GTEST: ")
    self.gtest_cb.select()
    self.gtest_cb.pack(side="left")

    self.tinyxml_cb = CompoundCheckbutton(self.embeddedlibs_frame, "TINYXML: ")
    self.tinyxml_cb.pack(side="left")

    self.rule3 = HRule(self.main_frame)
    self.rule3.pack()

  # additional linked libraries
    self.libraries_frame = Tkinter.Frame(self.main_frame)
    self.libraries_frame.pack(fill=Tkinter.X, side="top")

    self.libdirs_label = Tkinter.Label(
        self.libraries_frame,
        text="Extra lib dirs (space separated list): "
      )
    self.libdirs_label.pack(anchor="w")

    self.libdirs_entry = CompoundEntry(self.libraries_frame, "lib dirs: ", 
                                                                    width=60)
    self.libdirs_entry.pack(side="top")
    
    self.libraries_label = Tkinter.Label(
        self.libraries_frame,
        text="Libraries to link against (space separated list): "
      )
    self.libraries_label.pack(anchor="w")

    self.lib_entry = CompoundEntry(self.libraries_frame, "libs: ", width=60)
    self.lib_entry.pack(side="top")
    
    self.rule4 = HRule(self.main_frame)
    self.rule4.pack()

  # buttons and status
    # status
    self.status_text  = Tkinter.StringVar()
    self.status_text.set("Status: Please fill out project information.")
    self.status_msg = Tkinter.Message(
        textvar=self.status_text,
        width=400
        )
    self.status_msg.pack(side="left")

    # "generate" button
    self.accept_txt = Tkinter.StringVar()
    self.accept_txt.set("Generate")
    self.gen_button = Tkinter.Button(
        textvariable=self.accept_txt,
        command=self.genPkg)
    self.gen_button.pack(padx=8, pady=8, side="right")

    # "cancel" button
    self.cancel_button = Tkinter.Button(
        text="Cancel",
        command=quit)
    self.cancel_button.pack(padx=8, pady=8, side="right")
  
  def genPkg(self):
    if self.pkgname_entry.get() == "" or self.author_entry.get()=="":
      print "ERROR: please specify both a Package Name and Author"
      self.status_text.set("ERROR: Specify both a Package Name and Author")
      return

    if os.path.isdir(self.pkgname_entry.get()):
      print "ERROR: package directory already exists. Please choose a new name."
      self.status_text.set("ERROR: Package already exists... choose a new name")
      return

  
    self.status_text.set("Status: generating new package...\n")
    print "-- generating new package -- " 

    utils.genPkg_helper(self)

    self.status_text.set("Status: Finished!!! Thank you for playing")

    self.accept_txt.set("Finished")
    self.cancel_button.configure(state=Tkinter.DISABLED)
    self.gen_button.configure(command=quit)

if __name__ =='__main__':
  root=Tkinter.Tk()
  root.title("Generate a New RN Package")
  dlg = PkgGenDialog()

  root.mainloop()

