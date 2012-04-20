#CC = gcc
#CXX = g++
CC = icc
CXX = icpc

EXENAME = ../../bin/viewer
SRC = main.cpp
OBJ = $(SRC:%.cpp=%.o)
INCLUDEDIR = -I/usr/include/i386-linux-gnu `pkg-config opencv --cflags-only-I` `pkg-config gtk+-2.0 --cflags-only-I` -I/usr/include/flycapture -I/usr/include/triclops
LIBDIR = `pkg-config opencv --libs-only-L` `pkg-config gtk+-2.0 --libs-only-L` -L/usr/lib
LIB = `pkg-config opencv --libs-only-l` `pkg-config gtk+-2.0 --libs-only-l`
CFLAGS = $(INCLUDEDIR) -O2
LDFLAGS = $(LIBDIR) $(LIB)

$(EXENAME): $(OBJ)
	$(CC) -o $@ $(OBJ) $(CFLAGS) $(LDFLAGS)

.cpp.o:
	$(CXX) -o $@ $< -c $(CFLAGS)

clean:
	@rm -rf $(EXENAME) $(OBJ) *~ *.bak *.BAK

depend:
	makedepend -- $(CFLAGS) -- $(SRC)# DO NOT DELETE

main.o: /usr/include/gtk-2.0/gtk/gtk.h /usr/include/gtk-2.0/gdk/gdk.h
main.o: /usr/include/gtk-2.0/gdk/gdkcairo.h
main.o: /usr/include/gtk-2.0/gdk/gdkcolor.h /usr/include/cairo/cairo.h
main.o: /usr/include/cairo/cairo-features.h
main.o: /usr/include/gtk-2.0/gdk/gdktypes.h /usr/include/glib-2.0/glib.h
main.o: /usr/include/glib-2.0/glib/galloca.h
main.o: /usr/include/glib-2.0/glib/gtypes.h
main.o: /usr/lib64/glib-2.0/include/glibconfig.h
main.o: /usr/include/glib-2.0/glib/gmacros.h /usr/include/limits.h
main.o: /usr/include/features.h /usr/include/sys/cdefs.h
main.o: /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h
main.o: /usr/include/gnu/stubs-64.h /usr/include/bits/posix1_lim.h
main.o: /usr/include/bits/local_lim.h /usr/include/linux/limits.h
main.o: /usr/include/bits/posix2_lim.h /usr/include/glib-2.0/glib/garray.h
main.o: /usr/include/glib-2.0/glib/gasyncqueue.h
main.o: /usr/include/glib-2.0/glib/gthread.h
main.o: /usr/include/glib-2.0/glib/gerror.h
main.o: /usr/include/glib-2.0/glib/gquark.h
main.o: /usr/include/glib-2.0/glib/gatomic.h
main.o: /usr/include/glib-2.0/glib/gbacktrace.h
main.o: /usr/include/glib-2.0/glib/gbase64.h
main.o: /usr/include/glib-2.0/glib/gbookmarkfile.h /usr/include/time.h
main.o: /usr/include/bits/time.h /usr/include/bits/types.h
main.o: /usr/include/bits/typesizes.h /usr/include/glib-2.0/glib/gcache.h
main.o: /usr/include/glib-2.0/glib/glist.h /usr/include/glib-2.0/glib/gmem.h
main.o: /usr/include/glib-2.0/glib/gslice.h
main.o: /usr/include/glib-2.0/glib/gcompletion.h
main.o: /usr/include/glib-2.0/glib/gconvert.h
main.o: /usr/include/glib-2.0/glib/gdataset.h
main.o: /usr/include/glib-2.0/glib/gdate.h /usr/include/glib-2.0/glib/gdir.h
main.o: /usr/include/glib-2.0/glib/gfileutils.h
main.o: /usr/include/glib-2.0/glib/ghash.h /usr/include/glib-2.0/glib/ghook.h
main.o: /usr/include/glib-2.0/glib/giochannel.h
main.o: /usr/include/glib-2.0/glib/gmain.h
main.o: /usr/include/glib-2.0/glib/gslist.h
main.o: /usr/include/glib-2.0/glib/gstring.h
main.o: /usr/include/glib-2.0/glib/gunicode.h
main.o: /usr/include/glib-2.0/glib/gutils.h
main.o: /usr/include/glib-2.0/glib/gkeyfile.h
main.o: /usr/include/glib-2.0/glib/gmappedfile.h
main.o: /usr/include/glib-2.0/glib/gmarkup.h
main.o: /usr/include/glib-2.0/glib/gmessages.h
main.o: /usr/include/glib-2.0/glib/gnode.h
main.o: /usr/include/glib-2.0/glib/goption.h
main.o: /usr/include/glib-2.0/glib/gpattern.h
main.o: /usr/include/glib-2.0/glib/gprimes.h
main.o: /usr/include/glib-2.0/glib/gqsort.h
main.o: /usr/include/glib-2.0/glib/gqueue.h
main.o: /usr/include/glib-2.0/glib/grand.h /usr/include/glib-2.0/glib/grel.h
main.o: /usr/include/glib-2.0/glib/gscanner.h
main.o: /usr/include/glib-2.0/glib/gshell.h
main.o: /usr/include/glib-2.0/glib/gspawn.h
main.o: /usr/include/glib-2.0/glib/gstrfuncs.h
main.o: /usr/include/glib-2.0/glib/gthreadpool.h
main.o: /usr/include/glib-2.0/glib/gtimer.h
main.o: /usr/include/glib-2.0/glib/gtree.h
main.o: /usr/include/pango-1.0/pango/pango.h
main.o: /usr/include/pango-1.0/pango/pango-attributes.h
main.o: /usr/include/pango-1.0/pango/pango-font.h
main.o: /usr/include/pango-1.0/pango/pango-coverage.h
main.o: /usr/include/pango-1.0/pango/pango-types.h
main.o: /usr/include/glib-2.0/glib-object.h
main.o: /usr/include/glib-2.0/gobject/gboxed.h
main.o: /usr/include/glib-2.0/gobject/gtype.h
main.o: /usr/include/glib-2.0/gobject/genums.h
main.o: /usr/include/glib-2.0/gobject/gobject.h
main.o: /usr/include/glib-2.0/gobject/gvalue.h
main.o: /usr/include/glib-2.0/gobject/gparam.h
main.o: /usr/include/glib-2.0/gobject/gclosure.h
main.o: /usr/include/glib-2.0/gobject/gsignal.h
main.o: /usr/include/glib-2.0/gobject/gmarshal.h
main.o: /usr/include/glib-2.0/gobject/gparamspecs.h
main.o: /usr/include/glib-2.0/gobject/gsourceclosure.h
main.o: /usr/include/glib-2.0/gobject/gtypemodule.h
main.o: /usr/include/glib-2.0/gobject/gtypeplugin.h
main.o: /usr/include/glib-2.0/gobject/gvaluearray.h
main.o: /usr/include/glib-2.0/gobject/gvaluetypes.h
main.o: /usr/include/pango-1.0/pango/pango-break.h
main.o: /usr/include/pango-1.0/pango/pango-item.h
main.o: /usr/include/pango-1.0/pango/pango-context.h
main.o: /usr/include/pango-1.0/pango/pango-fontmap.h
main.o: /usr/include/pango-1.0/pango/pango-fontset.h
main.o: /usr/include/pango-1.0/pango/pango-engine.h
main.o: /usr/include/pango-1.0/pango/pango-glyph.h
main.o: /usr/include/pango-1.0/pango/pango-script.h
main.o: /usr/include/pango-1.0/pango/pango-enum-types.h
main.o: /usr/include/pango-1.0/pango/pango-layout.h
main.o: /usr/include/pango-1.0/pango/pango-glyph-item.h
main.o: /usr/include/pango-1.0/pango/pango-tabs.h
main.o: /usr/include/pango-1.0/pango/pango-renderer.h
main.o: /usr/lib64/gtk-2.0/include/gdkconfig.h
main.o: /usr/include/gtk-2.0/gdk/gdkpixbuf.h
main.o: /usr/include/gtk-2.0/gdk/gdkrgb.h
main.o: /usr/include/gtk-2.0/gdk-pixbuf/gdk-pixbuf.h
main.o: /usr/include/gtk-2.0/gdk-pixbuf/gdk-pixbuf-features.h
main.o: /usr/include/gtk-2.0/gdk-pixbuf/gdk-pixbuf-core.h
main.o: /usr/include/gtk-2.0/gdk-pixbuf/gdk-pixbuf-transform.h
main.o: /usr/include/gtk-2.0/gdk-pixbuf/gdk-pixbuf-animation.h
main.o: /usr/include/gtk-2.0/gdk-pixbuf/gdk-pixbuf-simple-anim.h
main.o: /usr/include/gtk-2.0/gdk-pixbuf/gdk-pixbuf-io.h /usr/include/stdio.h
main.o: /usr/include/libio.h /usr/include/_G_config.h /usr/include/wchar.h
main.o: /usr/include/bits/wchar.h /usr/include/gconv.h
main.o: /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h
main.o: /usr/include/glib-2.0/gmodule.h
main.o: /usr/include/gtk-2.0/gdk-pixbuf/gdk-pixbuf-loader.h
main.o: /usr/include/gtk-2.0/gdk-pixbuf/gdk-pixbuf-enum-types.h
main.o: /usr/include/pango-1.0/pango/pangocairo.h
main.o: /usr/include/gtk-2.0/gdk/gdkcursor.h
main.o: /usr/include/gtk-2.0/gdk/gdkdisplay.h
main.o: /usr/include/gtk-2.0/gdk/gdkevents.h
main.o: /usr/include/gtk-2.0/gdk/gdkdnd.h /usr/include/gtk-2.0/gdk/gdkinput.h
main.o: /usr/include/gtk-2.0/gdk/gdkdrawable.h
main.o: /usr/include/gtk-2.0/gdk/gdkgc.h
main.o: /usr/include/gtk-2.0/gdk/gdkenumtypes.h
main.o: /usr/include/gtk-2.0/gdk/gdkfont.h
main.o: /usr/include/gtk-2.0/gdk/gdkimage.h
main.o: /usr/include/gtk-2.0/gdk/gdkkeys.h
main.o: /usr/include/gtk-2.0/gdk/gdkdisplaymanager.h
main.o: /usr/include/gtk-2.0/gdk/gdkpango.h
main.o: /usr/include/gtk-2.0/gdk/gdkpixmap.h
main.o: /usr/include/gtk-2.0/gdk/gdkproperty.h
main.o: /usr/include/gtk-2.0/gdk/gdkregion.h
main.o: /usr/include/gtk-2.0/gdk/gdkscreen.h
main.o: /usr/include/gtk-2.0/gdk/gdkselection.h
main.o: /usr/include/gtk-2.0/gdk/gdkspawn.h
main.o: /usr/include/gtk-2.0/gdk/gdkvisual.h
main.o: /usr/include/gtk-2.0/gdk/gdkwindow.h
main.o: /usr/include/gtk-2.0/gtk/gtkaboutdialog.h
main.o: /usr/include/gtk-2.0/gtk/gtkdialog.h
main.o: /usr/include/gtk-2.0/gtk/gtkwindow.h
main.o: /usr/include/gtk-2.0/gtk/gtkaccelgroup.h
main.o: /usr/include/gtk-2.0/gtk/gtkenums.h /usr/include/gtk-2.0/gtk/gtkbin.h
main.o: /usr/include/gtk-2.0/gtk/gtkcontainer.h
main.o: /usr/include/gtk-2.0/gtk/gtkwidget.h
main.o: /usr/include/gtk-2.0/gtk/gtkobject.h
main.o: /usr/include/gtk-2.0/gtk/gtktypeutils.h
main.o: /usr/include/gtk-2.0/gtk/gtktypebuiltins.h
main.o: /usr/include/gtk-2.0/gtk/gtkdebug.h
main.o: /usr/include/gtk-2.0/gtk/gtkadjustment.h
main.o: /usr/include/gtk-2.0/gtk/gtkstyle.h
main.o: /usr/include/gtk-2.0/gtk/gtksettings.h
main.o: /usr/include/gtk-2.0/gtk/gtkrc.h /usr/include/atk-1.0/atk/atkobject.h
main.o: /usr/include/atk-1.0/atk/atkstate.h
main.o: /usr/include/atk-1.0/atk/atkrelationtype.h
main.o: /usr/include/gtk-2.0/gtk/gtkaccellabel.h
main.o: /usr/include/gtk-2.0/gtk/gtklabel.h
main.o: /usr/include/gtk-2.0/gtk/gtkmisc.h /usr/include/gtk-2.0/gtk/gtkmenu.h
main.o: /usr/include/gtk-2.0/gtk/gtkmenushell.h
main.o: /usr/include/gtk-2.0/gtk/gtkaccelmap.h
main.o: /usr/include/gtk-2.0/gtk/gtkaccessible.h
main.o: /usr/include/atk-1.0/atk/atk.h /usr/include/atk-1.0/atk/atkaction.h
main.o: /usr/include/atk-1.0/atk/atkcomponent.h
main.o: /usr/include/atk-1.0/atk/atkutil.h
main.o: /usr/include/atk-1.0/atk/atkdocument.h
main.o: /usr/include/atk-1.0/atk/atkeditabletext.h
main.o: /usr/include/atk-1.0/atk/atktext.h
main.o: /usr/include/atk-1.0/atk/atkgobjectaccessible.h
main.o: /usr/include/atk-1.0/atk/atkhyperlink.h
main.o: /usr/include/atk-1.0/atk/atkhyperlinkimpl.h
main.o: /usr/include/atk-1.0/atk/atkhypertext.h
main.o: /usr/include/atk-1.0/atk/atkimage.h
main.o: /usr/include/atk-1.0/atk/atknoopobject.h
main.o: /usr/include/atk-1.0/atk/atknoopobjectfactory.h
main.o: /usr/include/atk-1.0/atk/atkobjectfactory.h
main.o: /usr/include/atk-1.0/atk/atkregistry.h
main.o: /usr/include/atk-1.0/atk/atkobjectfactory.h
main.o: /usr/include/atk-1.0/atk/atkrelation.h
main.o: /usr/include/atk-1.0/atk/atkrelationset.h
main.o: /usr/include/atk-1.0/atk/atkselection.h
main.o: /usr/include/atk-1.0/atk/atkstateset.h
main.o: /usr/include/atk-1.0/atk/atkstreamablecontent.h
main.o: /usr/include/atk-1.0/atk/atktable.h
main.o: /usr/include/atk-1.0/atk/atkvalue.h
main.o: /usr/include/gtk-2.0/gtk/gtkaction.h
main.o: /usr/include/gtk-2.0/gtk/gtkactiongroup.h
main.o: /usr/include/gtk-2.0/gtk/gtkitemfactory.h
main.o: /usr/include/gtk-2.0/gtk/gtkalignment.h
main.o: /usr/include/gtk-2.0/gtk/gtkarrow.h
main.o: /usr/include/gtk-2.0/gtk/gtkaspectframe.h
main.o: /usr/include/gtk-2.0/gtk/gtkframe.h
main.o: /usr/include/gtk-2.0/gtk/gtkassistant.h
main.o: /usr/include/gtk-2.0/gtk/gtkbbox.h /usr/include/gtk-2.0/gtk/gtkbox.h
main.o: /usr/include/gtk-2.0/gtk/gtkbindings.h
main.o: /usr/include/gtk-2.0/gtk/gtkbutton.h
main.o: /usr/include/gtk-2.0/gtk/gtkimage.h
main.o: /usr/include/gtk-2.0/gtk/gtkcalendar.h
main.o: /usr/include/gtk-2.0/gtk/gtksignal.h
main.o: /usr/include/gtk-2.0/gtk/gtkmarshal.h
main.o: /usr/include/gtk-2.0/gtk/gtkcelllayout.h
main.o: /usr/include/gtk-2.0/gtk/gtkcellrenderer.h
main.o: /usr/include/gtk-2.0/gtk/gtkcelleditable.h
main.o: /usr/include/gtk-2.0/gtk/gtktreeviewcolumn.h
main.o: /usr/include/gtk-2.0/gtk/gtktreemodel.h
main.o: /usr/include/gtk-2.0/gtk/gtktreesortable.h
main.o: /usr/include/gtk-2.0/gtk/gtkcellrendereraccel.h
main.o: /usr/include/gtk-2.0/gtk/gtkcellrenderertext.h
main.o: /usr/include/gtk-2.0/gtk/gtkcellrenderercombo.h
main.o: /usr/include/gtk-2.0/gtk/gtkcellrenderertext.h
main.o: /usr/include/gtk-2.0/gtk/gtkcellrendererpixbuf.h
main.o: /usr/include/gtk-2.0/gtk/gtkcellrendererprogress.h
main.o: /usr/include/gtk-2.0/gtk/gtkcellrendererspin.h
main.o: /usr/include/gtk-2.0/gtk/gtkcellrenderertoggle.h
main.o: /usr/include/gtk-2.0/gtk/gtkcellview.h
main.o: /usr/include/gtk-2.0/gtk/gtkcheckbutton.h
main.o: /usr/include/gtk-2.0/gtk/gtktogglebutton.h
main.o: /usr/include/gtk-2.0/gtk/gtkcheckmenuitem.h
main.o: /usr/include/gtk-2.0/gtk/gtkmenuitem.h
main.o: /usr/include/gtk-2.0/gtk/gtkitem.h
main.o: /usr/include/gtk-2.0/gtk/gtkclipboard.h
main.o: /usr/include/gtk-2.0/gtk/gtkselection.h
main.o: /usr/include/gtk-2.0/gtk/gtktextiter.h
main.o: /usr/include/gtk-2.0/gtk/gtktexttag.h
main.o: /usr/include/gtk-2.0/gtk/gtktextchild.h
main.o: /usr/include/gtk-2.0/gtk/gtkclist.h
main.o: /usr/include/gtk-2.0/gtk/gtkhscrollbar.h
main.o: /usr/include/gtk-2.0/gtk/gtkscrollbar.h
main.o: /usr/include/gtk-2.0/gtk/gtkrange.h
main.o: /usr/include/gtk-2.0/gtk/gtkvscrollbar.h
main.o: /usr/include/gtk-2.0/gtk/gtkcolorbutton.h
main.o: /usr/include/gtk-2.0/gtk/gtkcolorsel.h
main.o: /usr/include/gtk-2.0/gtk/gtkvbox.h
main.o: /usr/include/gtk-2.0/gtk/gtkcolorseldialog.h
main.o: /usr/include/gtk-2.0/gtk/gtkcombo.h
main.o: /usr/include/gtk-2.0/gtk/gtkhbox.h
main.o: /usr/include/gtk-2.0/gtk/gtkcombobox.h
main.o: /usr/include/gtk-2.0/gtk/gtktreeview.h
main.o: /usr/include/gtk-2.0/gtk/gtkdnd.h /usr/include/gtk-2.0/gtk/gtkentry.h
main.o: /usr/include/gtk-2.0/gtk/gtkeditable.h
main.o: /usr/include/gtk-2.0/gtk/gtkimcontext.h
main.o: /usr/include/gtk-2.0/gtk/gtkentrycompletion.h
main.o: /usr/include/gtk-2.0/gtk/gtkliststore.h
main.o: /usr/include/gtk-2.0/gtk/gtktreemodelfilter.h
main.o: /usr/include/gtk-2.0/gtk/gtkcomboboxentry.h
main.o: /usr/include/gtk-2.0/gtk/gtkctree.h
main.o: /usr/include/gtk-2.0/gtk/gtkcurve.h
main.o: /usr/include/gtk-2.0/gtk/gtkdrawingarea.h
main.o: /usr/include/gtk-2.0/gtk/gtkeventbox.h
main.o: /usr/include/gtk-2.0/gtk/gtkexpander.h
main.o: /usr/include/gtk-2.0/gtk/gtkfilesel.h
main.o: /usr/include/gtk-2.0/gtk/gtkfixed.h
main.o: /usr/include/gtk-2.0/gtk/gtkfilechooserbutton.h
main.o: /usr/include/gtk-2.0/gtk/gtkfilechooser.h
main.o: /usr/include/gtk-2.0/gtk/gtkfilefilter.h
main.o: /usr/include/gtk-2.0/gtk/gtkfilechooserdialog.h
main.o: /usr/include/gtk-2.0/gtk/gtkfilechooser.h
main.o: /usr/include/gtk-2.0/gtk/gtkfilechooserwidget.h
main.o: /usr/include/gtk-2.0/gtk/gtkfontbutton.h
main.o: /usr/include/gtk-2.0/gtk/gtkfontsel.h
main.o: /usr/include/gtk-2.0/gtk/gtkgamma.h /usr/include/gtk-2.0/gtk/gtkgc.h
main.o: /usr/include/gtk-2.0/gtk/gtkhandlebox.h
main.o: /usr/include/gtk-2.0/gtk/gtkhbbox.h
main.o: /usr/include/gtk-2.0/gtk/gtkhpaned.h
main.o: /usr/include/gtk-2.0/gtk/gtkpaned.h
main.o: /usr/include/gtk-2.0/gtk/gtkhruler.h
main.o: /usr/include/gtk-2.0/gtk/gtkruler.h
main.o: /usr/include/gtk-2.0/gtk/gtkhscale.h
main.o: /usr/include/gtk-2.0/gtk/gtkscale.h
main.o: /usr/include/gtk-2.0/gtk/gtkhseparator.h
main.o: /usr/include/gtk-2.0/gtk/gtkseparator.h
main.o: /usr/include/gtk-2.0/gtk/gtkiconfactory.h
main.o: /usr/include/gtk-2.0/gtk/gtkicontheme.h
main.o: /usr/include/gtk-2.0/gtk/gtkiconview.h
main.o: /usr/include/gtk-2.0/gtk/gtkimagemenuitem.h
main.o: /usr/include/gtk-2.0/gtk/gtkimcontextsimple.h
main.o: /usr/include/gtk-2.0/gtk/gtkimmulticontext.h
main.o: /usr/include/gtk-2.0/gtk/gtkinputdialog.h
main.o: /usr/include/gtk-2.0/gtk/gtkinvisible.h
main.o: /usr/include/gtk-2.0/gtk/gtklayout.h
main.o: /usr/include/gtk-2.0/gtk/gtklinkbutton.h
main.o: /usr/include/gtk-2.0/gtk/gtklist.h
main.o: /usr/include/gtk-2.0/gtk/gtklistitem.h
main.o: /usr/include/gtk-2.0/gtk/gtkmain.h
main.o: /usr/include/gtk-2.0/gtk/gtkmenubar.h
main.o: /usr/include/gtk-2.0/gtk/gtkmenutoolbutton.h
main.o: /usr/include/gtk-2.0/gtk/gtkmenu.h
main.o: /usr/include/gtk-2.0/gtk/gtktoolbutton.h
main.o: /usr/include/gtk-2.0/gtk/gtktoolitem.h
main.o: /usr/include/gtk-2.0/gtk/gtktooltips.h
main.o: /usr/include/gtk-2.0/gtk/gtkmessagedialog.h
main.o: /usr/include/gtk-2.0/gtk/gtkmodules.h
main.o: /usr/include/gtk-2.0/gtk/gtknotebook.h
main.o: /usr/include/gtk-2.0/gtk/gtkoldeditable.h
main.o: /usr/include/gtk-2.0/gtk/gtkoptionmenu.h
main.o: /usr/include/gtk-2.0/gtk/gtkpixmap.h
main.o: /usr/include/gtk-2.0/gtk/gtkplug.h
main.o: /usr/include/gtk-2.0/gtk/gtksocket.h
main.o: /usr/include/gtk-2.0/gtk/gtkpreview.h
main.o: /usr/include/gtk-2.0/gtk/gtkprintoperation.h
main.o: /usr/include/gtk-2.0/gtk/gtkmain.h
main.o: /usr/include/gtk-2.0/gtk/gtkenums.h
main.o: /usr/include/gtk-2.0/gtk/gtkwindow.h
main.o: /usr/include/gtk-2.0/gtk/gtkpagesetup.h
main.o: /usr/include/gtk-2.0/gtk/gtkpapersize.h
main.o: /usr/include/gtk-2.0/gtk/gtkprintsettings.h
main.o: /usr/include/gtk-2.0/gtk/gtkprintcontext.h
main.o: /usr/include/gtk-2.0/gtk/gtkprintoperationpreview.h
main.o: /usr/include/gtk-2.0/gtk/gtkprogress.h
main.o: /usr/include/gtk-2.0/gtk/gtkprogressbar.h
main.o: /usr/include/gtk-2.0/gtk/gtkradioaction.h
main.o: /usr/include/gtk-2.0/gtk/gtktoggleaction.h
main.o: /usr/include/gtk-2.0/gtk/gtkradiobutton.h
main.o: /usr/include/gtk-2.0/gtk/gtkradiomenuitem.h
main.o: /usr/include/gtk-2.0/gtk/gtkradiotoolbutton.h
main.o: /usr/include/gtk-2.0/gtk/gtktoggletoolbutton.h
main.o: /usr/include/gtk-2.0/gtk/gtktoolbutton.h
main.o: /usr/include/gtk-2.0/gtk/gtkrecentchooser.h
main.o: /usr/include/gtk-2.0/gtk/gtkrecentmanager.h
main.o: /usr/include/gtk-2.0/gtk/gtkrecentfilter.h
main.o: /usr/include/gtk-2.0/gtk/gtkrecentchooserdialog.h
main.o: /usr/include/gtk-2.0/gtk/gtkrecentchooser.h
main.o: /usr/include/gtk-2.0/gtk/gtkrecentchoosermenu.h
main.o: /usr/include/gtk-2.0/gtk/gtkrecentchooserwidget.h
main.o: /usr/include/gtk-2.0/gtk/gtkrecentfilter.h
main.o: /usr/include/gtk-2.0/gtk/gtkrecentmanager.h
main.o: /usr/include/gtk-2.0/gtk/gtkscrolledwindow.h
main.o: /usr/include/gtk-2.0/gtk/gtkviewport.h
main.o: /usr/include/gtk-2.0/gtk/gtkseparatormenuitem.h
main.o: /usr/include/gtk-2.0/gtk/gtkseparatortoolitem.h
main.o: /usr/include/gtk-2.0/gtk/gtksizegroup.h
main.o: /usr/include/gtk-2.0/gtk/gtkspinbutton.h
main.o: /usr/include/gtk-2.0/gtk/gtkstatusbar.h
main.o: /usr/include/gtk-2.0/gtk/gtkstatusicon.h
main.o: /usr/include/gtk-2.0/gtk/gtkstock.h
main.o: /usr/include/gtk-2.0/gtk/gtktable.h
main.o: /usr/include/gtk-2.0/gtk/gtktearoffmenuitem.h
main.o: /usr/include/gtk-2.0/gtk/gtktext.h
main.o: /usr/include/gtk-2.0/gtk/gtktextbuffer.h
main.o: /usr/include/gtk-2.0/gtk/gtktexttagtable.h
main.o: /usr/include/gtk-2.0/gtk/gtktextmark.h
main.o: /usr/include/gtk-2.0/gtk/gtktextbufferrichtext.h
main.o: /usr/include/gtk-2.0/gtk/gtktextview.h
main.o: /usr/include/gtk-2.0/gtk/gtktipsquery.h
main.o: /usr/include/gtk-2.0/gtk/gtktoggletoolbutton.h
main.o: /usr/include/gtk-2.0/gtk/gtktoolbar.h
main.o: /usr/include/gtk-2.0/gtk/gtktoolitem.h
main.o: /usr/include/gtk-2.0/gtk/gtktree.h
main.o: /usr/include/gtk-2.0/gtk/gtktreednd.h
main.o: /usr/include/gtk-2.0/gtk/gtktreeitem.h
main.o: /usr/include/gtk-2.0/gtk/gtktreemodelsort.h
main.o: /usr/include/gtk-2.0/gtk/gtktreeselection.h
main.o: /usr/include/gtk-2.0/gtk/gtktreestore.h
main.o: /usr/include/gtk-2.0/gtk/gtkuimanager.h
main.o: /usr/include/gtk-2.0/gtk/gtkvbbox.h
main.o: /usr/include/gtk-2.0/gtk/gtkversion.h
main.o: /usr/include/gtk-2.0/gtk/gtkvpaned.h
main.o: /usr/include/gtk-2.0/gtk/gtkvruler.h
main.o: /usr/include/gtk-2.0/gtk/gtkvscale.h
main.o: /usr/include/gtk-2.0/gtk/gtkvseparator.h ../humantracking.h
main.o: /usr/local/include/opencv/cv.h
main.o: /usr/local/include/opencv2/core/core_c.h
main.o: /usr/local/include/opencv2/core/types_c.h /usr/include/assert.h
main.o: /usr/include/stdlib.h /usr/include/sys/types.h /usr/include/endian.h
main.o: /usr/include/bits/endian.h /usr/include/sys/select.h
main.o: /usr/include/bits/select.h /usr/include/bits/sigset.h
main.o: /usr/include/sys/sysmacros.h /usr/include/bits/pthreadtypes.h
main.o: /usr/include/alloca.h /usr/include/string.h /usr/include/stdint.h
main.o: /usr/include/math.h /usr/include/bits/huge_val.h
main.o: /usr/include/bits/mathdef.h /usr/include/bits/mathcalls.h
main.o: /usr/local/include/opencv2/core/core.hpp
main.o: /usr/local/include/opencv2/core/version.hpp
main.o: /usr/local/include/opencv2/core/operations.hpp
main.o: /usr/local/include/opencv2/core/mat.hpp
main.o: /usr/local/include/opencv2/imgproc/imgproc_c.h
main.o: /usr/local/include/opencv2/imgproc/types_c.h
main.o: /usr/local/include/opencv2/imgproc/imgproc.hpp
main.o: /usr/local/include/opencv2/video/tracking.hpp
main.o: /usr/local/include/opencv2/features2d/features2d.hpp
main.o: /usr/local/include/opencv2/flann/miniflann.hpp
main.o: /usr/local/include/opencv2/flann/flann.hpp
main.o: /usr/local/include/opencv2/calib3d/calib3d.hpp
main.o: /usr/local/include/opencv2/objdetect/objdetect.hpp
main.o: /usr/local/include/opencv2/legacy/compat.hpp
