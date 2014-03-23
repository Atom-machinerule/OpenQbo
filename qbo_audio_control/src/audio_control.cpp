/*
*Copyright 2004-2007 Lennart Poettering <mzihzrgre (at) 0pointer (dot) de>
*Copyright (C) 2012 Thecorpora Inc.
*
*This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.
*
*This program is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
*
*You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*/



#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <vector> 
#include <string>
#include "ros/ros.h"
#include <pulse/pulseaudio.h>
#include <stdlib.h>
#include <sstream>


/*
static void timeval_add_usec(struct timeval *tv, pa_usec_t v) {
    uint32_t sec = v/1000000;
    tv->tv_sec += sec;
    v -= sec*1000000;
    
    tv->tv_usec += v;

    while (tv->tv_usec >= 1000000) {
        tv->tv_sec++;
        tv->tv_usec -= 1000000;
    }
}*/

static pa_context *context = NULL;
static pa_stream *stream = NULL;
static enum {
    PLAYBACK,
    RECORD
} mode = PLAYBACK;//RECORD;


int mic_on=-1;


//Values modified in the config launcher


double threshold_to_on=0.001; 
double threshold_to_off=0.01;
int max_accum=5;
int mic_volume=58;
int front_volume=100;
int master_front_volume=20;
int mic_boost=0;
int speaker=100;

std::string default_input="Front Mic";

std::vector <float> accum_values (max_accum,0);




/*
void show_error(const char *txt, bool show_pa_error = true) {
    char buf[256];

    if (show_pa_error)
        snprintf(buf, sizeof(buf), "%s: %s", txt, pa_strerror(pa_context_errno(context)));
    
}*/

static void stream_update_timing_info_callback(pa_stream *s, int success, void *) {
    pa_usec_t t;
    int negative = 0;
    
    if (!success || pa_stream_get_latency(s, &t, &negative) < 0) {
        //show_error("Failed to get latency information");
        return;
    }

}
/*
static gboolean latency_func(gpointer) {
    pa_operation *o;
    
    if (!stream)
        return false;

    if (!(o = pa_stream_update_timing_info(stream, stream_update_timing_info_callback, NULL)))
        g_message("pa_stream_update_timing_info() failed: %s", pa_strerror(pa_context_errno(context)));
    else
        pa_operation_unref(o);
    
    return true;
}*/

static void stream_read_callback(pa_stream *s, size_t l, void *) {
    if (! ros::ok())
    {
        exit(-1);
    }
    const void *p;

    if (pa_stream_peek(s, &p, &l) < 0) {
//        g_message("pa_stream_peek() failed: %s", pa_strerror(pa_context_errno(context)));
        return;
    }
    float * tmp = (float*)p;
    float v = fabs(tmp[0]);
    float accum=0;
    for (int i=max_accum-1;i>0;i--)
    {
        accum_values[i]=accum_values[i-1];
        accum=accum+accum_values[i];
//        printf("i=%d;accumulate=%f\n",i,accum);
    }
    accum_values[0]=v;
    accum=accum+accum_values[0];
//    printf("Volume:%f\n",v);
//    printf("Accumulate:%f\n",accum);
    accum=accum/max_accum;
//    printf("Mid:%f\n",accum);
    if (accum>threshold_to_off && mic_on!=0)
    {
    	long int secs;
    	secs=time(NULL);
	mic_on=0;
	system("amixer set Capture nocap");// > /dev/null");
    }
    else if (accum<=threshold_to_on && mic_on!=1)
    {
	mic_on=1;
	system("amixer set Capture cap");// > /dev/null");
    }
   pa_stream_drop(s);

}

static void stream_state_callback(pa_stream *s, void *) {
    switch (pa_stream_get_state(s)) {
        case PA_STREAM_UNCONNECTED:
        case PA_STREAM_CREATING:
            break;

        case PA_STREAM_READY:
//            g_timeout_add(100, latency_func, NULL);
            pa_operation_unref(pa_stream_update_timing_info(stream, stream_update_timing_info_callback, NULL));
            break;
            
        case PA_STREAM_FAILED:
           // show_error("Connection failed");
            break;
            
        case PA_STREAM_TERMINATED:
	    exit(-1);
    }
}

static void create_stream(const char *name, const char *description, const pa_sample_spec &ss, const pa_channel_map &cmap) {
    pa_sample_spec nss;

   // g_free(device_name);
   // device_name = g_strdup(name);
  //  g_free(device_description);
  //  device_description = g_strdup(description);
    
    nss.format = PA_SAMPLE_FLOAT32;
    nss.rate = ss.rate;
    nss.channels = ss.channels;
    
   // g_message("Using sample format: %s", pa_sample_spec_snprint(t, sizeof(t), &nss));
   // g_message("Using channel map: %s", pa_channel_map_snprint(t, sizeof(t), &cmap));

    stream = pa_stream_new(context, "PulseAudio Volume Meter", &nss, &cmap);
    pa_stream_set_state_callback(stream, stream_state_callback, NULL);
    pa_stream_set_read_callback(stream, stream_read_callback, NULL);
    pa_stream_connect_record(stream, name, NULL, (enum pa_stream_flags) 0);
}
/*
static void context_get_source_info_callback(pa_context *, const pa_source_info *si, int is_last, void *) {
    if (is_last < 0) {
        show_error("Failed to get source information");
        return;
    }

    if (!si)
        return;
    g_message("1");
    //create_stream(si->name, si->description, si->sample_spec, si->channel_map);
}
*/
static void context_get_sink_info_callback(pa_context *, const pa_sink_info *si, int is_last, void *) {
    if (is_last < 0) {
       // show_error("Failed to get sink information");
        return;
    }

    if (!si)
        return;
    create_stream(si->monitor_source_name, si->description, si->sample_spec, si->channel_map);
}

static void context_get_server_info_callback(pa_context *c, const pa_server_info*si, void *) {
    if (!si) {
       // show_error("Failed to get server information");
        return;
    }
/*
    if (mode == PLAYBACK) {

        if (!si->default_sink_name) {
            show_error("No default sink set.", false);
            return;
        }
	g_message("3");
        pa_operation_unref(pa_context_get_sink_info_by_name(c, si->default_sink_name, context_get_sink_info_callback, NULL));
        
    } else if (mode == RECORD) {

        if (!si->default_source_name) {
            show_error("No default source set.", false);
            return;
        }

//        pa_operation_unref(pa_context_get_source_info_by_name(c, si->default_source_name, context_get_source_info_callback, NULL));
    }*/
    if (!si->default_sink_name) {
       // show_error("No default sink set.", false);
        return;
    }
    pa_operation_unref(pa_context_get_sink_info_by_name(c, si->default_sink_name, context_get_sink_info_callback, NULL));

}

static void context_state_callback(pa_context *c, void *) {
    switch (pa_context_get_state(c)) {
        case PA_CONTEXT_UNCONNECTED:
        case PA_CONTEXT_CONNECTING:
        case PA_CONTEXT_AUTHORIZING:
        case PA_CONTEXT_SETTING_NAME:
            break;

        case PA_CONTEXT_READY:
	    pa_operation_unref(pa_context_get_server_info(c, context_get_server_info_callback, NULL));
/*            if (device_name && mode == RECORD){
	
                pa_operation_unref(pa_context_get_source_info_by_name(c, device_name, context_get_source_info_callback, NULL));
		g_message("44");
	    }
            else if (device_name && mode == PLAYBACK)
	    {
		g_message("4");
                pa_operation_unref(pa_context_get_sink_info_by_name(c, device_name, context_get_sink_info_callback, NULL));
		}
            else
		g_message("5");
                pa_operation_unref(pa_context_get_server_info(c, context_get_server_info_callback, NULL));
            
            break;
            pa_operation_unref(pa_context_get_server_info(c, context_get_server_info_callback, NULL));*/
	
            
        case PA_CONTEXT_FAILED:
           // show_error("Connection failed");
            break;
            
        case PA_CONTEXT_TERMINATED:
	   break;
           // Gtk::Main::quit();
    }
}

void read_parameters(){
    ros::NodeHandle nh("~");
    nh.getParam("threshold_to_on",threshold_to_on);
    nh.getParam("threshold_to_off",threshold_to_off);
    nh.getParam("max_accum",max_accum);
    nh.getParam("default_input",default_input);
    nh.getParam("mic_volume",mic_volume);
    nh.getParam("mic_boost",mic_boost);
    nh.getParam("front_volume",front_volume);
    nh.getParam("master_front_volume",master_front_volume);
    nh.getParam("speaker",speaker);

    ROS_INFO("Threshold_to_on:%f",threshold_to_on);
    ROS_INFO("Threshold_to_off:%f",threshold_to_off);
    ROS_INFO("Max_accum:%d",max_accum);
    ROS_INFO("Default_input:%s",default_input.c_str());
    ROS_INFO("Mic_volume:%d%%", mic_volume);
    ROS_INFO("Mic_boost_volume:%d%%", mic_boost);
    ROS_INFO("Front_volume:%d%%", front_volume);
    ROS_INFO("Master_front_volume:%d%%", master_front_volume);
    ROS_INFO("Speaker volume %d%%",speaker);

    std::string instruction;
    instruction="amixer set 'Input Source' ";
    instruction=instruction+" '"+default_input+"'";
    system(instruction.c_str());

    std::ostringstream volume_string;

    volume_string << mic_volume;
    instruction="amixer set 'Capture' "+volume_string.str()+"%";
    system(instruction.c_str());

    volume_string.str("");
    volume_string << mic_boost;
    instruction="amixer set 'Front Mic Boost' "+volume_string.str()+"%";
    system(instruction.c_str());

    volume_string.str("");
    volume_string << front_volume;
    instruction="amixer set 'Front' "+volume_string.str()+"%";
    system(instruction.c_str());

    volume_string.str("");
    volume_string << master_front_volume;
    instruction="amixer set 'Master' "+volume_string.str()+"%" + volume_string.str()+"%";
    system(instruction.c_str());

    volume_string.str("");
    volume_string << speaker;
    instruction="amixer set 'Speaker' "+volume_string.str()+"%" + volume_string.str()+"%";
    system(instruction.c_str());

}

void read_parameters_callback(const ros::TimerEvent&){
    read_parameters();
}

int main(int argc, char *argv[]) {

    ros::init(argc, argv , "audio_control");
    ros::NodeHandle n("/audio_control");
    read_parameters();
    ros::Timer timer = n.createTimer(ros::Duration(4), read_parameters_callback);
    ros::AsyncSpinner spinner(1); 
    spinner.start();

    bool record = false;
    pa_mainloop *m;

    signal(SIGPIPE, SIG_IGN);

    mode = record ? RECORD : PLAYBACK;
    
    m = pa_mainloop_new();
    context = pa_context_new(pa_mainloop_get_api(m), "PulseAudio Ros Controller");

    pa_context_set_state_callback(context, context_state_callback, NULL);
    pa_context_connect(context, NULL, PA_CONTEXT_NOAUTOSPAWN, NULL);


    int * returned;
    pa_mainloop_run(m, returned);
            
    if (stream)
        pa_stream_unref(stream);
    if (context)
        pa_context_unref(context);



    return 0;
}
