import os, re, fileinput
from datetime import datetime as date

# DHP - this should not be a hard coded value :(
pkg_template_dir = "/prj/tools/templates/pkg_template"

def genPkg_helper(pkgdata):
  printPkgInfo(pkgdata)
  pkgname=re.sub(" ", "_", pkgdata.pkgname_entry.get())
  owner=pkgdata.owner_entry.get()
  website=pkgdata.website_entry.get()
  author=pkgdata.author_entry.get()
  email=pkgdata.email_entry.get()
  desc=pkgdata.desc_entry.get()
  license=pkgdata.license.get()
  libs=pkgdata.lib_entry.get()
  libdirs=pkgdata.libdirs_entry.get()

  cmd = "cp -r " + pkg_template_dir + " " + pkgname
  os.system(cmd)

  cmd = "rm -rf `find " + pkgname + " -name .svn` "
  os.system(cmd)

  subdirs   = ""
  swsubdirs = ""

  if pkgdata.sw_cb.get():
    subdirs = subdirs + " sw"  
  else:
    cmd = "rm -rf " + pkgname + "/sw"
    os.system(cmd)
    
  if pkgdata.fw_cb.get():
    subdirs = subdirs + " fw"  
  else:
    cmd = "rm -rf " + pkgname + "/fw"
    os.system(cmd)
    
  if pkgdata.hw_cb.get():
    subdirs = subdirs
  else:
    cmd = "rm -rf " + pkgname + "/hw"
    os.system(cmd)

  if pkgdata.gtest_cb.get():
    swsubdirs = swsubdirs + " gtest"
  else:
    cmd = "rm -rf " + pkgname + "/sw/gtest*"
    os.system(cmd)
    cmd = "rm -rf " + pkgname + "/include/gtest*"
    os.system(cmd)
    cmd = "rm -rf " + pkgname + "/docs/doxy/x_gtest.doxy*"
    os.system(cmd)

  if pkgdata.tinyxml_cb.get():
    swsubdirs = swsubdirs + " tinyxml"
  else:
    cmd = "rm -rf " + pkgname + "/sw/tiny*"
    os.system(cmd)
    cmd = "rm -rf " + pkgname + "/include/tiny*"
    os.system(cmd)


# eula
  for line in fileinput.FileInput(pkgname+"/EULA.txt", inplace=True):
    line = line.replace("@PKGNAME@", pkgname)
    line = line.replace("@DATE@", str(date.now().year))
    line = line.replace("@OWNER@", owner)
    line = line.replace("@WEBSITE@", website)
    print line,

# readme  
  for line in fileinput.FileInput(pkgname+"/README.xml", inplace=True):
    line = line.replace("@PKGNAME@", pkgname)
    line = line.replace("@DATE@", str(date.now().year))
    line = line.replace("@OWNER@", owner)
    line = line.replace("@WEBSITE@", website)
    line = line.replace("@DESC@", desc)
    print line,

# Makefile  
  for line in fileinput.FileInput(pkgname+"/Makefile", inplace=True):
    line = line.replace("@PKGNAME@", pkgname)
    line = line.replace("@DATE@", str(date.now().year))
    line = line.replace("@OWNER@", owner)
    line = line.replace("@WEBSITE@", website)
    line = line.replace("@AUTHOR@", author)
    line = line.replace("@EMAIL@", email)
    line = line.replace("@SUBDIRS@", subdirs)
    print line,

# examples/Makefile  
  for line in fileinput.FileInput(pkgname+"/examples/Makefile", inplace=True):
    line = line.replace("@PKGNAME@", pkgname)
    line = line.replace("@DATE@", str(date.now().year))
    line = line.replace("@OWNER@", owner)
    line = line.replace("@WEBSITE@", website)
    line = line.replace("@AUTHOR@", author)
    line = line.replace("@EMAIL@", email)
    line = line.replace("@SUBDIRS@", subdirs)
    print line,

# sw/Makefile  
  if pkgdata.sw_cb.get():
    for line in fileinput.FileInput(pkgname+"/sw/Makefile", inplace=True):
      line = line.replace("@PKGNAME@", pkgname)
      line = line.replace("@DATE@", str(date.now().year))
      line = line.replace("@OWNER@", owner)
      line = line.replace("@WEBSITE@", website)
      line = line.replace("@AUTHOR@", author)
      line = line.replace("@EMAIL@", email)
      line = line.replace("@SUBDIRS@", swsubdirs)
      print line,

# fw/Makefile  
  if pkgdata.sw_cb.get():
    for line in fileinput.FileInput(pkgname+"/fw/Makefile", inplace=True):
      line = line.replace("@PKGNAME@", pkgname)
      line = line.replace("@DATE@", str(date.now().year))
      line = line.replace("@OWNER@", owner)
      line = line.replace("@WEBSITE@", website)
      line = line.replace("@AUTHOR@", author)
      line = line.replace("@EMAIL@", email)
      line = line.replace("@SUBDIRS@", swsubdirs)
      print line,

# gtest
  if pkgdata.gtest_cb.get():
    for line in fileinput.FileInput(pkgname+"/docs/doxy/x_gtest.doxy",
                                                      inplace=True):
      line = line.replace("@PKGNAME@", pkgname)
      line = line.replace("@DATE@", str(date.now().year))
      line = line.replace("@OWNER@", owner)
      line = line.replace("@WEBSITE@", website)
      line = line.replace("@AUTHOR@", author)
      line = line.replace("@EMAIL@", email)
      line = line.replace("@SUBDIRS@", swsubdirs)
      print line,

# doxy includes
  for line in fileinput.FileInput(pkgname+"/docs/doxy/zModDoxyIncludes.doxy",
                                                    inplace=True):
    line = line.replace("@PKGNAME@", pkgname)
    line = line.replace("@DATE@", str(date.now().year))
    line = line.replace("@OWNER@", owner)
    line = line.replace("@WEBSITE@", website)
    line = line.replace("@AUTHOR@", author)
    line = line.replace("@EMAIL@", email)
    line = line.replace("@SUBDIRS@", swsubdirs)
    print line,

# docs/doxy/main.doxy
  for line in fileinput.FileInput(pkgname+"/docs/doxy/main.doxy", 
                                                     inplace=True):
    line = line.replace("@PKGNAME@", pkgname)
    line = line.replace("@DATE@", str(date.now().year))
    line = line.replace("@OWNER@", owner)
    line = line.replace("@WEBSITE@", website)
    line = line.replace("@AUTHOR@", author)
    line = line.replace("@EMAIL@", email)
    line = line.replace("@SUBDIRS@", swsubdirs)
    print line,

# docs/doxy/page_EULA.doxy
  for line in fileinput.FileInput(pkgname+"/docs/doxy/page_EULA.doxy", 
                                                     inplace=True):
    line = line.replace("@PKGNAME@", pkgname)
    line = line.replace("@DATE@", str(date.now().year))
    line = line.replace("@OWNER@", owner)
    line = line.replace("@WEBSITE@", website)
    line = line.replace("@AUTHOR@", author)
    line = line.replace("@EMAIL@", email)
    line = line.replace("@SUBDIRS@", swsubdirs)
    print line,

# make/utenv.sh
  for line in fileinput.FileInput(pkgname+"/make/utenv.sh", inplace=True):
    line = line.replace("@PKGNAME@", pkgname)
    line = line.replace("@DATE@", str(date.now().year))
    line = line.replace("@OWNER@", owner)
    line = line.replace("@WEBSITE@", website)
    line = line.replace("@AUTHOR@", author)
    line = line.replace("@EMAIL@", email)
    line = line.replace("@SUBDIRS@", swsubdirs)
    print line,

# make/Pkg.mk
  for line in fileinput.FileInput(pkgname+"/make/Pkg.mk", inplace=True):
    line = line.replace("@PKGNAME@", pkgname)
    line = line.replace("@DATE@", str(date.now().year))
    line = line.replace("@OWNER@", owner)
    line = line.replace("@WEBSITE@", website)
    line = line.replace("@AUTHOR@", author)
    line = line.replace("@EMAIL@", email)
    line = line.replace("@LIBS@", libs)
    line = line.replace("@LIBDIRS@", libdirs)
    print line,

# make/doxy.conf
  for line in fileinput.FileInput(pkgname+"/make/doxy.conf", inplace=True):
    line = line.replace("@PKGNAME@", pkgname)
    line = line.replace("@DATE@", str(date.now().year))
    line = line.replace("@OWNER@", owner)
    line = line.replace("@WEBSITE@", website)
    line = line.replace("@AUTHOR@", author)
    line = line.replace("@EMAIL@", email)
    line = line.replace("@LIBS@", libs)
    line = line.replace("@LIBDIRS@", libdirs)
    print line,

# if making deb packages...
  if pkgdata.dpkg_cb.get():
  # make/deb-dev/control
    for line in fileinput.FileInput(pkgname+"/make/deb-dev/control", 
                                                         inplace=True):
      line = line.replace("@PKGNAME@", pkgname)
      line = line.replace("@DATE@", str(date.now().year))
      line = line.replace("@OWNER@", owner)
      line = line.replace("@WEBSITE@", website)
      line = line.replace("@AUTHOR@", author)
      line = line.replace("@EMAIL@", email)
      line = line.replace("@LIBS@", libs)
      line = line.replace("@LIBDIRS@", libdirs)
      print line,

  # make/deb-src/control
    for line in fileinput.FileInput(pkgname+"/make/deb-src/control", 
                                                         inplace=True):
      line = line.replace("@PKGNAME@", pkgname)
      line = line.replace("@DATE@", str(date.now().year))
      line = line.replace("@OWNER@", owner)
      line = line.replace("@WEBSITE@", website)
      line = line.replace("@AUTHOR@", author)
      line = line.replace("@EMAIL@", email)
      print line,

  # make/deb-doc/control
    for line in fileinput.FileInput(pkgname+"/make/deb-doc/control", 
                                                         inplace=True):
      line = line.replace("@PKGNAME@", pkgname)
      line = line.replace("@DATE@", str(date.now().year))
      line = line.replace("@OWNER@", owner)
      line = line.replace("@WEBSITE@", website)
      line = line.replace("@AUTHOR@", author)
      line = line.replace("@EMAIL@", email)
      print line,

  # make/deb-dev/prerm
    for line in fileinput.FileInput(pkgname+"/make/deb-dev/prerm", 
                                                         inplace=True):
      line = line.replace("@PKGNAME@", pkgname)
      line = line.replace("@DATE@", str(date.now().year))
      line = line.replace("@OWNER@", owner)
      line = line.replace("@WEBSITE@", website)
      line = line.replace("@AUTHOR@", author)
      line = line.replace("@EMAIL@", email)
      print line,

  # make/deb-src/prerm
    for line in fileinput.FileInput(pkgname+"/make/deb-src/prerm", 
                                                         inplace=True):
      line = line.replace("@PKGNAME@", pkgname)
      line = line.replace("@DATE@", str(date.now().year))
      line = line.replace("@OWNER@", owner)
      line = line.replace("@WEBSITE@", website)
      line = line.replace("@AUTHOR@", author)
      line = line.replace("@EMAIL@", email)
      print line,

  # make/deb-doc/prerm
    for line in fileinput.FileInput(pkgname+"/make/deb-doc/prerm", 
                                                         inplace=True):
      line = line.replace("@PKGNAME@", pkgname)
      line = line.replace("@DATE@", str(date.now().year))
      line = line.replace("@OWNER@", owner)
      line = line.replace("@WEBSITE@", website)
      line = line.replace("@AUTHOR@", author)
      line = line.replace("@EMAIL@", email)
      print line,

  # make/deb-dev/postinst
    for line in fileinput.FileInput(pkgname+"/make/deb-dev/postinst", 
                                                         inplace=True):
      line = line.replace("@PKGNAME@", pkgname)
      line = line.replace("@DATE@", str(date.now().year))
      line = line.replace("@OWNER@", owner)
      line = line.replace("@WEBSITE@", website)
      line = line.replace("@AUTHOR@", author)
      line = line.replace("@EMAIL@", email)
      print line,

  # make/deb-src/postinst
    for line in fileinput.FileInput(pkgname+"/make/deb-src/postinst", 
                                                         inplace=True):
      line = line.replace("@PKGNAME@", pkgname)
      line = line.replace("@DATE@", str(date.now().year))
      line = line.replace("@OWNER@", owner)
      line = line.replace("@WEBSITE@", website)
      line = line.replace("@AUTHOR@", author)
      line = line.replace("@EMAIL@", email)
      print line,

  # make/deb-doc/postinst
    for line in fileinput.FileInput(pkgname+"/make/deb-doc/postinst", 
                                                         inplace=True):
      line = line.replace("@PKGNAME@", pkgname)
      line = line.replace("@DATE@", str(date.now().year))
      line = line.replace("@OWNER@", owner)
      line = line.replace("@WEBSITE@", website)
      line = line.replace("@AUTHOR@", author)
      line = line.replace("@EMAIL@", email)
      print line,
  else: # no deb pkgs
    cmd = "rm -rf " + pkgname + "/make/deb-*"
    os.system(cmd)

  cmd = "/prj/tools/eula_update.py rnrestricted " + pkgname
  os.system(cmd)

def printPkgInfo(pkgdata):
  print "  Package Name : " + re.sub(" ", "_", pkgdata.pkgname_entry.get());
  print "  Owner        : " + pkgdata.owner_entry.get() + \
      " (" + pkgdata.website_entry.get() + ")"
  print "  Author       : " + pkgdata.author_entry.get() + \
      " <" + pkgdata.email_entry.get() + ">"
  print "  License      : " + pkgdata.license.get()

  print "  Package Features: "
  i=0
  if pkgdata.sw_cb.get() == 1 :
    print "    * sw "
    i=i+1
  if pkgdata.hw_cb.get() == 1 :
    print "    * hw "
    i=i+1
  if pkgdata.fw_cb.get() == 1 :
    print "    * fw "
    i=i+1
  if pkgdata.dpkg_cb.get() == 1 :
    print "    * dpkg "
    i=i+1
  if i==0:
    print "     None!"
  
  print "  3rd party libs (to be embedded in package): "
  i=0
  if pkgdata.gtest_cb.get() == 1 :
    print "    * gtest "
    i=i+1
  if pkgdata.tinyxml_cb.get() == 1 :
    print "    * tinyxml "
    i=i+1
  if i == 0:
    print "     None!"
