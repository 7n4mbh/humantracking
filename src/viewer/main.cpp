#include <iostream>
#include <string>
#include <sstream>

#include <gtk/gtk.h>

#include "../humantracking.h"

using namespace std;


GtkWidget *window;
GtkWidget *progress;
GtkWidget *planview;

unsigned long long t_start_tracking, t_end_tracking;

void MessageReceived( std::string msg )
{
    if( msg.empty() ) {
        return;
    }

    vector<string> strCmd;
    char* cstrcmd = new char[ msg.size() + 1 ];
    strcpy( cstrcmd, msg.c_str() );
        
    char* tp = strtok( cstrcmd, " =," );
    strCmd.push_back( string( tp ) );
    while ( tp != NULL ) {
        tp = strtok( NULL, " =," );
        if ( tp != NULL ) {
            strCmd.push_back( string( tp ) );
        }
    }

    delete [] cstrcmd;

    if( strCmd.empty() ) {
        return;
    }

    if( strCmd[ 0 ] == "TrackingBlock" ) {
        istringstream iss;
        iss.str( strCmd[ 1 ] );
        iss >> t_start_tracking;
        iss.clear();
        iss.str( strCmd[ 2 ] );
        iss >> t_end_tracking;
    }

    gtk_widget_queue_draw( window );
}


/*
#ifdef WINDOWS_OS
DWORD WINAPI KeyInThread( LPVOID p_param )
#else
void* KeyInThread( void* p_param )
#endif
{
    string str;

    for( ; ; ) {
        cin >> str;

        gchar* str_msg = (gchar*)str.c_str();
        gtk_statusbar_push( GTK_STATUSBAR(window)
                          , gtk_statusbar_get_context_id( GTK_STATUSBAR(window), str_msg )
                          , str_msg );

        MessageReceived( str );

    }
#ifdef WINDOWS_OS
    return 1;
#endif
#ifdef LINUX_OS
    return NULL;
#endif
}
*/

static gboolean signal_key_press( GtkWidget* widget, GdkEventKey* evt, gpointer window )
{
    static ostringstream oss;

    if( evt->hardware_keycode == '\r' ) {
        gchar* str_msg;
        str_msg = g_strdup_printf( "%s", oss.str().c_str() );
        gtk_statusbar_push( GTK_STATUSBAR(window)
                            , gtk_statusbar_get_context_id( GTK_STATUSBAR(window), str_msg )
                            , str_msg );

        MessageReceived( oss.str() );

        oss.str( "" );
        oss.clear();
        g_free( str_msg );
    } else {
        if( evt->length > 0 ) {
            oss << (char)evt->keyval;
        }
    }

    return FALSE;
}

static gboolean on_expose_event(GtkWidget *widget, GdkEventExpose *evt, gpointer data)
{
  cairo_t *cr;

  cr = gdk_cairo_create(progress->window/*widget->window*/);

  //cairo_move_to(cr, 10, 10);
  //cairo_show_text(cr, "test");

  cairo_set_source_rgb(cr, 1, 0, 0);
  cairo_set_line_width (cr, 1.0);
  cairo_rectangle(cr, 5, 5, progress->allocation.width / 2, progress->allocation.height / 2);
  cairo_fill(cr);
  
  cairo_stroke(cr);

  cairo_destroy(cr);


    //gdk_draw_arc( progress->window
    //            , progress->style->fg_gc[gtk_widget_get_state(progress)]
    //            , TRUE
    //            , 0, 0
    //            , progress->allocation.width, progress->allocation.height
    //            , 0, 64 * 360 );

    return FALSE;
}

void init_gui( int argc, char *argv[] )
{
  //GtkWidget *window;
  GtkWidget *vbox;

  GtkWidget *statusbar;
  //GtkWidget *button;


  gtk_init(&argc, &argv);

  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);
  gtk_window_set_default_size(GTK_WINDOW(window), 250, 200);
  gtk_window_set_title(GTK_WINDOW(window), "Viewer");

  vbox = gtk_vbox_new(FALSE, 0);
  gtk_container_add(GTK_CONTAINER(window), vbox);

  statusbar = gtk_statusbar_new();
  gtk_box_pack_end(GTK_BOX(vbox), statusbar, FALSE, TRUE, 1);

  progress = gtk_drawing_area_new();
  gtk_widget_set_size_request(progress, 80, 35);
  gtk_box_pack_end(GTK_BOX(vbox), progress, FALSE, TRUE, 1);

  //button = gtk_button_new_with_label("button");
  //gtk_widget_set_size_request(button, 80, 35);
  //gtk_box_pack_end(GTK_BOX(vbox), button, TRUE/*FALSE*/, TRUE, 1);

  planview = gtk_image_new();
  gtk_box_pack_end(GTK_BOX(vbox), planview, TRUE, TRUE, 1);
  
  g_signal_connect( G_OBJECT(window)
                  , "key-press-event"
                  , G_CALLBACK (signal_key_press)
                  , G_OBJECT(statusbar) );
  g_signal_connect(progress, "expose-event",
      G_CALLBACK(on_expose_event), NULL);
  g_signal_connect_swapped(G_OBJECT(window), "destroy",
        G_CALLBACK(gtk_main_quit), NULL);

  gtk_widget_show((GtkWidget*)statusbar);
  gtk_widget_show_all(window);

  //GdkImage* gdk_image;
  //GdkBitmap* mask;
  //gtk_image_get_image((GtkImage*)progress, &gdk_image, &mask);
}

int main( int argc, char *argv[] )
{
//#ifdef WINDOWS_OS
//    DWORD idThread;
//    HANDLE hThread = CreateThread( NULL, 0, KeyInThread, NULL, NULL, &idThread );
//#else
//    pthread_t thread;
//    pthread_create(&thread , NULL , KeyInThread , NULL);
//#endif

    init_gui( argc, argv );
    gtk_main();
//
//#ifdef WINDOWS_OS
//    //WaitForSingleObject( hThread, INFINITE );
//    TerminateThread( hThread, FALSE );
//#else
//    pthread_join(thread , NULL);
//#endif

    return 0;
}