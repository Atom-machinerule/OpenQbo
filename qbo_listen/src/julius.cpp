/*
*Copyright (c) 2012-2013 TheCorpora SL
*
*This program is free software: 
*you can redistribute it and/or modify it under the terms of the GNU General Public License 
*as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
*This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
*without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
*See the GNU General Public License for more details.
*You should have received a copy of the GNU General Public License along with this program. 
*If not, see <http://www.gnu.org/licenses/>.
*
*Under Section 7 of GPL version 3, you are granted additional
*permissions described in the GCC Runtime Library Exception, version
*3.1, as published by the Free Software Foundation.
*/

#include <julius/juliuslib.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include "qbo_listen/Listened.h"
#include <sstream>
#include <stdio.h>
#include <ros/package.h>


using namespace std;
const float indiv_threshold = 0.09;
const float threshold = 0.5;
Recog *recog;
JCONF_SEARCH *sr;
char * DefaultSR;
vector<ros::Publisher> topic_vector;
bool pause_recog=false;



void callback_engine_pause_func(Recog *recog, void *data)
{
  pause_recog=true;
//  j_request_pause (recog);
//  j_request_resume(recog);
}


/*
left trim
*/
void lTrim(string& str) {
  string::size_type pos = 0;
  while (pos < str.size() && isspace(str[pos])) pos++;
  str.erase(0, pos);
}

/*
Rigth trim
*/ 
void rTrim(string& str) {
  string::size_type pos = str.size();
  while (pos > 0 && isspace(str[pos - 1])) pos--;
  str.erase(pos);
}

/*
Left and rigth trim
*/ 
void bTrim(string& str) {
  lTrim(str);
  rTrim(str);
}

/*
Desactivate an SR
Actually not used
*/
void deactivateSR(char *search)
{
 ROS_INFO("Deactiviting: %s\n", search);
 j_process_deactivate(recog, search);
}

/*
Desactivate all SR
Actuallly not used
*/
void deactivateAll_SR()
{
 ROS_WARN("Deactiviting all grammars");
 JCONF_SEARCH *sr_tmp;
 sr_tmp=sr;
 while (sr_tmp!=NULL)
 {
  deactivateSR(sr_tmp->name);
  sr_tmp=sr_tmp->next;
 }
}

/*
Activate SR
Actually not used
*/
void activateSR(char *search)
{
 //deactivate_ALL_SR();
 ROS_INFO("Activiting: %s\n", search);
 j_process_activate(recog, search);
}

/*
Get SR name from topic name
*/
char * topicToSr(char *topic_name)
{
 char * SR_name;
 SR_name = strtok(topic_name, "/");
 SR_name = strtok(NULL, "/");
 return SR_name;
}
/*
Deactivate an SR by topic name
*/
void deactivateSrByTopic(char *topic_name)
{
//  ROS_INFO("Topic to deactivate:%s",topicToSr(topic_name));
  deactivateSR(topicToSr(topic_name));
}

/*
Activate an SR by topic name
*/
void activateSrByTopic(char *topic_name)
{
//  ROS_INFO("Topic to activate:%s",topicToSr(topic_name));
 activateSR(topicToSr(topic_name));
}


/*
Activate-Deactivate grammars without subscribers
*/
void grammarControllerCallback(const ros::TimerEvent&)
{
  vector<ros::Publisher>::const_iterator iter;
  int i;
  char * topic_name = (char *)malloc(512);
  vector<ros::Publisher> chatter(topic_vector.size());
  copy(topic_vector.begin(), topic_vector.end(), chatter.begin()); 
  int SRnum=(int) chatter.size();
  for (i=0;i<(int) chatter.size();i++)
  {
    if (i==0)
    {
       iter = chatter.begin();
    }
    else
    {
       iter +=1;
    }
    strcpy(topic_name, iter->getTopic().c_str());
    if (iter->getNumSubscribers()==0)
    {
         deactivateSrByTopic(topic_name); 
    //     ROS_INFO("Deactivated grammar: %s",topic_name);
         SRnum=SRnum-1;
    }
    else
    {
         activateSrByTopic(topic_name);
      //   ROS_INFO("Activated grammar: %s",topic_name);
    }
  }
  if (SRnum>0 && pause_recog==true)
  {
     j_request_resume (recog);
     pause_recog=false;
  }
}




/*
Add grammars topics
*/
void createPublishers(vector<ros::Publisher> & listen_topics, ros::NodeHandle & n)
{
 JCONF_SEARCH *sr_tmp;
 sr_tmp=sr;
 char * topname = (char *)malloc(512);
 strcpy(topname,"/listen/");
 string topicname;
 while (sr_tmp!=NULL)
 {
  strcat(topname,sr_tmp->name);
  listen_topics.push_back(n.advertise<qbo_listen::Listened>(topname, 10));
  sr_tmp=sr_tmp->next;
  strcpy(topname,"/listen/");
 }
 ROS_INFO("Created %d grammars publishers",listen_topics.size());
}


/*
Main Callback that run for each audio input
*/
static void 
outputResult(Recog *recog, void *dummy)
{
  int i;
  WORD_INFO *winfo;
  WORD_ID *seq;
  int seqnum;
  int n;
  Sentence *s;
  RecogProcess *r;
  JCONF_SEARCH *sr_tmp;
  
  string sent;
  string word;
  string allscore;
  float totalscore;
  float minscore;
  std::stringstream ss2;
  vector<ros::Publisher> * chatter = static_cast<vector<ros::Publisher>*>(dummy);
  qbo_listen::Listened msg;

  vector<ros::Publisher>::const_iterator iter;
  char complete_name[25];

  if (! ros::ok())
  {
        exit(0);
  }
  sr_tmp=sr;
  for(r=recog->process_list;r;r=r->next) {

    /* skip the process if the process is not alive */
    if (! r->live) continue;

    if (r->result.status < 0) {      /* no results obtained */
      /* outout message according to the status code */
      switch(r->result.status) {
      case J_RESULT_STATUS_REJECT_POWER:
        ROS_WARN("<input rejected by power>\n");
        break;
      case J_RESULT_STATUS_TERMINATE:
        ROS_WARN("<input teminated by request>\n");
        break;
      case J_RESULT_STATUS_ONLY_SILENCE:
        ROS_WARN("<input rejected by decoder (silence input result)>\n");
        break;
      case J_RESULT_STATUS_REJECT_GMM:
        ROS_WARN("<input rejected by GMM>\n");
        break;
      case J_RESULT_STATUS_REJECT_SHORT:
        ROS_WARN("<input rejected by short input>\n");
        break;
      case J_RESULT_STATUS_FAIL:
        ROS_WARN("<search failed>\n");
        break;
      }
      /* continue to next process instance */
      continue;
    }

    /* output results for all the obtained sentences */
    winfo = r->lm->winfo;

    /* For each audio there will generate one sentece for each grammar. 
    This loop is for manage each one*/
    for(n = 0; n < r->result.sentnum; n++) {
      s = &(r->result.sent[n]); 
      for (i=0;i<(int)chatter->size();i++)
      {
           if (i==0)
           {
               iter = chatter->begin();
           }
           else
           {
               iter +=1;
           }
           strcpy(complete_name,"/listen/");
           strcat(complete_name,r->config->name);//sr_tmp->name);//s->gram_id);
//           ROS_WARN("Sr_tmp id: %d  - Recogprocces grammar name: %s - Chatter size:%d -Comparando esto %s con: %s",sr_tmp->id,r->config->name,(int)chatter->size(),complete_name,iter->getTopic().c_str());
           if (strcmp(complete_name,iter->getTopic().c_str())==0)
           {
               break;
           }
      }
//      s = &(r->result.sent[n]);
      seq = s->word;
      seqnum = s->word_num;
      sent = "";
      word = "";
      for(i=0;i<seqnum;i++){
         word=winfo->woutput[seq[i]];
         if (word.compare("<s>") !=0 && word.compare("</s>")!=0)
         {
           sent = sent + " " + word;
         }
      }
      bTrim(sent);
      ROS_INFO("Sentence:%s",sent.c_str());
      
      allscore="";
      totalscore=0;
      minscore=1;
      ss2.str("");  
      for (i=0;i<seqnum; i++){
        // ROS_INFO("Score aqui %f",s->confidence[i]);
         if (i!=0 and i!= seqnum-1)
         {
             if (minscore>s->confidence[i])
             {
                 minscore=s->confidence[i];
             }
             totalscore=totalscore+s->confidence[i];
             ss2 << s->confidence[i];
             ss2 << " ";
         }
      }
      allscore=ss2.str();
      ROS_INFO("CMScore:%s",allscore.c_str());
      ROS_INFO("Grammar: %s", sr_tmp->name);
      totalscore=totalscore/(seqnum-2);
      msg.total_score=totalscore;
      msg.all_score=allscore;
      msg.min_score=minscore;
      if (totalscore<threshold || minscore<indiv_threshold)
      {
        ROS_WARN("Message don't pass the threshold");
        msg.msg="";
        msg.not_msg=sent.c_str();
      } 
      else
      {
        ROS_INFO("Message pass the threshold");
        msg.not_msg="";
        msg.msg=sent.c_str();
      }
      ROS_INFO("Message published");
      iter->publish(msg);
      sr_tmp=sr_tmp->next;
    }
  }

  fflush(stdout);
}



/**
 * Main function
 * 
 */

int
main(int argc, char *argv[])
{


  ros::init(argc, argv , "qbo_audio_listener");
  ros::NodeHandle n("/listen");
  Jconf *jconf;
  
//  signal(SIGTERM,signalCapture);
//  signal(SIGINT,signalCapture);
  
  static char speechfilename[MAXPATHLEN];

  int ret; 
  
  string param1,param2;
  int numargs=3;
  char * arguments[numargs];

  if (argc==1)
  {
    ros::NodeHandle nh("~");
    nh.getParam("configfile", param2);
    nh.getParam("config", param1);
    if (param1=="" || param2=="")
    {
      param1="-C";
      param2 = ros::package::getPath("qbo_listen")+"/config/julius.jconf";
      arguments[0]=argv[0];
      arguments[1]=(char *)param1.c_str();
      arguments[2]=(char *)param2.c_str();
    }
    ROS_WARN("Config file: %s",param2.c_str());
    arguments[0]=argv[0];
    arguments[1]=(char *)param1.c_str();
    arguments[2]=(char *)param2.c_str();
    jconf = j_config_load_args_new(numargs,arguments);
  }
  else if (argc==2)
  {
    param1="-C";
    param2 =argv[1];
    arguments[0]=argv[0];
    arguments[1]=(char *)param1.c_str();
    arguments[2]=(char *)param2.c_str();
    ROS_WARN("Config file: %s",param2.c_str());
    jconf = j_config_load_args_new(numargs,arguments);
  }
  else if (argc==3)
  {
    param2=argv[2];
    ROS_WARN("Config file: %s",param2.c_str());
    jconf = j_config_load_args_new(argc, argv);
  }
  else
  {
    jconf = j_config_load_args_new(argc, argv);
  }

  if (jconf == NULL) {                /* error */
    ROS_ERROR("Parameters error.\n");
    ROS_ERROR("Unable to run the node, parameters needed, please use the launcher or review julius documentation for parameters info");
    return -1;
  }

  
  recog = j_create_instance_from_jconf(jconf);
  if (recog == NULL) {
    ROS_ERROR("Error in startup\n");
    return -1;
  }



//  vector<ros::Publisher> topic_vector;
  sr=jconf->search_root;
  createPublishers(topic_vector,n);

  callback_add(recog, CALLBACK_RESULT, outputResult, &topic_vector);
  callback_add(recog, CALLBACK_PAUSE_FUNCTION, callback_engine_pause_func,NULL);
//Timer
//  ros::Timer timer = n.createTimer(ros::Duration(1), grammarControllerCallback);
//  ros::AsyncSpinner spinner(1);
//  spinner.start();

  if (j_adin_init(recog) == FALSE) {    /* error */
    return -1;
  }

//  This line is to see julius more info
//  j_recog_info(recog);

  /***********************************/
  /* Open input stream and recognize */
  /***********************************/
  if (jconf->input.speech_input == SP_MFCFILE) {

    while (get_line_from_stdin(speechfilename, MAXPATHLEN, (char *)"enter MFCC filename->") != NULL) {
      if (verbose_flag) ROS_INFO("\ninput MFCC file: %s\n", speechfilename);
      /* open the input file */
      ret = j_open_stream(recog, speechfilename);
      switch(ret) {
      case 0:			/* succeeded */
	break;
      case -1:      		/* error */
	/* go on to the next input */
	continue;
      case -2:			/* end of recognition */
	return 0;
      }
      /* recognition loop */
      ret = j_recognize_stream(recog);
      if (ret == -1) return -1;	/* error */
      /* reach here when an input ends */
    }

  } else {
    /* raw speech input (microphone etc.) */
   // deactivate_ALL_SR(recog, "prueba1");

    switch(j_open_stream(recog, NULL)) {
    case 0:			/* succeeded */
      break;
    case -1:      		/* error */
      ROS_ERROR("Error in input stream");
      return 0;
    case -2:			/* end of recognition process */
      ROS_ERROR("Failed to begin input stream");
      return 0;
    }
       
 
    /**********************/
    /* Recognization Loop */
    /**********************/
    /* enter main loop to recognize the input stream */
    /* finish after whole input has been processed and input reaches end */
    ros::spinOnce();

    ret = j_recognize_stream(recog);
    if (ret == -1) return -1;	/* error */
    
    /*******/
    /* End */
    /*******/
  }

  j_close_stream(recog);
  j_recog_free(recog);
  return(0);
}
