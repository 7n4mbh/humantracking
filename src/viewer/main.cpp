#include <iostream>
#include <string>
#include <sstream>

#include <gtk/gtk.h>

#include "../humantracking.h"

using namespace std;


GtkWidget *window;
GtkWidget *progress;
GtkWidget *planview;
GtkWidget *statusbar;

vector<unsigned long long> tracking_block;
unsigned long long t_start_tracking;
vector<unsigned long long> pepmap;
int tracking_status = 0;
unsigned long long t_current_time = 0;

static gboolean on_expose_event(GtkWidget *widget, GdkEventExpose *evt, gpointer data);

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
    
    unsigned long long t_start, t_end;
    if( strCmd[ 0 ] == "StartTime" ) {
        istringstream iss;
	unsigned long long t;
        iss.str( strCmd[ 1 ] );
        iss >> t_start_tracking;
    } else if( strCmd[ 0 ] == "TrackingBlock" ) {
        istringstream iss;
        iss.str( strCmd[ 1 ] );
        iss >> t_start;
        iss.clear();
        iss.str( strCmd[ 2 ] );
        iss >> t_end;
	if( tracking_block.empty() ) {
	  t_start_tracking = t_start;
	}
	tracking_block.push_back( t_start );
	tracking_block.push_back( t_end );
	//cout << "New Tracking Block Added:" << t_start << "-" << t_end << endl;
    } else if( strCmd[ 0 ] == "TrackingStatus" ) {
        istringstream iss;
        iss.str( strCmd[ 1 ] );
        iss >> tracking_status;       
    } else if( strCmd[ 0 ] == "PEPMap" ) {
        istringstream iss;
	    unsigned long long t;
        iss.str( strCmd[ 1 ] );
        iss >> t;
        pepmap.push_back( t );
    } else if( strCmd[ 0 ] == "Time" ) {
        istringstream iss;
	    unsigned long long t;
        iss.str( strCmd[ 1 ] );
        iss >> t_current_time;
    }
    gtk_widget_queue_draw( progress );
    //on_expose_event( progress, NULL, NULL );
}



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
        gtk_statusbar_push( GTK_STATUSBAR(statusbar/*window*/)
			    , gtk_statusbar_get_context_id( GTK_STATUSBAR(statusbar/*window*/), str_msg )
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

/*
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
*/

static gboolean on_expose_event(GtkWidget *widget, GdkEventExpose *evt, gpointer data)
{
  cairo_t *cr;

  cr = gdk_cairo_create(progress->window/*widget->window*/);

  //cairo_move_to(cr, 10, 10);
  //cairo_show_text(cr, "test");

  cairo_set_source_rgb(cr, 1, 0, 0);
  cairo_set_line_width (cr, 1.0);

  const unsigned long long display_term = 60000000ULL;
  const double t2px = (double)progress->allocation.width / (double)display_term;


  // Draw Current time
  {
    const int x = (int)( (double)( t_current_time - t_start_tracking ) * t2px );
    cairo_set_source_rgb(cr, 1, 0, 0);
    cairo_move_to(cr, x, 0 );
    cairo_line_to(cr, x, 30 );
    cairo_stroke( cr );
  }

  // Draw PEP-map time
  for( int i = 0; i < pepmap.size(); ++i ) {
    const int x = (int)( (double)( pepmap[ i ] - t_start_tracking ) * t2px );
    cairo_set_source_rgb(cr, 0, 0, 0);
    cairo_move_to(cr, x, 0 );
    cairo_line_to(cr, x, 20 );
    cairo_stroke( cr );
  }

  // Draw tracking blocks
  for( int i = 0; i < tracking_block.size() / 2; ++i ) {
    const int x1 = (int)( (double)( tracking_block[ 2 * i     ] - t_start_tracking ) * t2px );
    const int x2 = (int)( (double)( tracking_block[ 2 * i + 1 ] - t_start_tracking ) * t2px );

    if( i >= tracking_block.size() / 2 - 1 ) {
      switch( tracking_status ) {
      case 0:
  	  cairo_set_source_rgb(cr, 1, 1, 1); // Making trajectories
	  break;
      case 1:
	  cairo_set_source_rgb(cr, 1, 0, 0); // Clustering
	  break;
      case 2:
	  cairo_set_source_rgb(cr, 1, 1, 0); // Renovating
	  break;
      default:
	  cairo_set_source_rgb(cr, 0, 1, 0);
	  break;
      }
    } else {
      cairo_set_source_rgb(cr, 0, 1, 0);
    }
    cairo_rectangle(cr, x1, 5, x2 - x1, 10 );
    cairo_fill(cr);
   //cairo_stroke(cr);

    cairo_set_source_rgb(cr, 0, 0, 0);
    cairo_rectangle(cr, x1, 5, x2 - x1, 10 );
    cairo_stroke(cr);
  }

  cairo_destroy(cr);


    //gdk_draw_arc( progress->window
    //            , progress->style->fg_gc[gtk_widget_get_state(progress)]
    //            , TRUE
    //            , 0, 0
    //            , progress->allocation.width, progress->allocation.height
    //            , 0, 64 * 360 );

    return FALSE;
}

static gboolean
time_handler(GtkWidget *widget)
{
  if (widget->window == NULL) return FALSE;

  gtk_widget_queue_draw(widget);
  return TRUE;
}

void init_gui( int argc, char *argv[] )
{
  //GtkWidget *window;
  GtkWidget *vbox;

  //GtkWidget *statusbar;
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
  
  //g_signal_connect( G_OBJECT(window)
  //                , "key-press-event"
  //                , G_CALLBACK (signal_key_press)
  //                , G_OBJECT(statusbar) );
  g_signal_connect(progress, "expose-event",
      G_CALLBACK(on_expose_event), NULL);
  g_signal_connect_swapped(G_OBJECT(window), "destroy",
        G_CALLBACK(gtk_main_quit), NULL);

  g_timeout_add(100, (GSourceFunc) time_handler, (gpointer) window);

  gtk_widget_show((GtkWidget*)statusbar);
  gtk_widget_show_all(window);

  //GdkImage* gdk_image;
  //GdkBitmap* mask;
  //gtk_image_get_image((GtkImage*)progress, &gdk_image, &mask);
}

int main( int argc, char *argv[] )
{
#ifdef WINDOWS_OS
    DWORD idThread;
    HANDLE hThread = CreateThread( NULL, 0, KeyInThread, NULL, NULL, &idThread );
#else
    pthread_t thread;
    pthread_create(&thread , NULL , KeyInThread , NULL);
#endif

    init_gui( argc, argv );
    gtk_main();

#ifdef WINDOWS_OS
    //WaitForSingleObject( hThread, INFINITE );
    TerminateThread( hThread, FALSE );
#else
    pthread_cancel(thread);
#endif

    return 0;
}
