/*
 * Software License Agreement (GPLv2 License)
 * 
 * Copyright (c) 2012 TheCorpora SL
 *
 * This program is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public License as 
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, 
 * MA 02110-1301, USA.
 *
 * Authors: Miguel Angel Julian <miguel.julian@openqbo.com>;
 *          Daniel Cuadrado <daniel.cuadrado@openqbo.com>;
 *          Arturo Bajuelos <arturo@openqbo.com>; 
 *          Sergio Merino <s.merino@openqbo.com>;
 */
var keys = {};
var leftButtonDown = false;
var imgPort="8081";


var loop;

var quality=50;
var widthCamera=320;
var heightCamera=240;

var last_time_pressed=0;
var movement_mode = 0;
var TIME_THRESHOLD=300;
var fps=24;

var ctx;
var img = new Image();
var canvas;

var action;
var countdown;
var bool_drawing=false;
var recording = false;
var watching = false;
var training = false;
var auxTime4Coundown;

var name2Learn;

var objectORface="object";


var msgWebCamWatching = "Watching";
var msgWebCamTraining = "Training...";
var msgWebCam = "";
var msgWebCamRec = "Rec";

var cameraURL=""

var base_move_timer;


var LINEAR_SPEED = 0.3;
var ANGULAR_SPEED = 1.0;

tabUnload = function(){
    clearInterval(loop); 
    imgSrc = jQuery("#iframeTeleoperation").attr("src");
    stopCmd = imgSrc.replace("stream","stop");
    jQuery.get(stopCmd);
};

function move_timer(line, ang)
{
    input = {"line":line,"angu":ang};
    jQuery.post('/teleoperation/move',input,function(data){
    });
}


function startEverything(){


    output = {image:"live_leftEye", quality: quality, width: widthCamera, height: heightCamera};
    jQuery.post('/mjpegServer/getUrlFrom',output,function(data) {
        cameraURL=data+"&t="+new Date().getTime();
        jQuery("#iframeTeleoperation").attr("src",cameraURL);
    });
    
    originX=jQuery("#video2").offset().left;
    originY=jQuery("#video2").offset().top;


    jQuery("#video2").draggable({
	    drag: function() {


   			originX=jQuery("#video").offset().left+(jQuery("#video").width()/2);
    			originY=jQuery("#video").offset().top+(jQuery("#video").height()/2);

			x = jQuery("#video2").offset().left+(jQuery("#video2").width()/2);
                        y = jQuery("#video2").offset().top+(jQuery("#video2").height()/2);

			trueX = x-originX;
			trueY = y-originY;		
	
            input = {"yaw":-trueX,"pitch":trueY};
            jQuery.post('/teleoperation/head',input, function(data){
            });
                     
	    }
        
        ,
            stop: function() {
		jQuery("#video2").offset({"top":jQuery("#video").offset().top,"left":jQuery("#video").offset().left});		
 		input = {"yaw":0,"pitch":0};
                jQuery.post('/teleoperation/head',input, function(data){
                });
            }
    });


    jQuery('#video2').dblclick(function() {
        
        jQuery.post('/teleoperation/head_to_zero_position', function(data){
                });
    });

	//Buttons detection
	jQuery("#forward").mousedown(function(){
        line=0.2;
        ang = 0.0;
        base_move_timer=setInterval("move_timer("+line+","+ang+")", 100);
	}).mouseup(function(){
        clearInterval(base_move_timer);
        input = {"line":0.0,"angu":0.0};
        jQuery.post('/teleoperation/move',input,function(data){
        });
	});  
    


	//Buttons detection
	jQuery("#back").mousedown(function(){
        line=-0.2;
        ang = 0.0;
        base_move_timer=setInterval("move_timer("+line+","+ang+")", 100);
	}).mouseup(function(){
        clearInterval(base_move_timer);
        input = {"line":0.0,"angu":0.0};
        jQuery.post('/teleoperation/move',input,function(data){
        });
	}); 

	//Buttons detection
	jQuery("#left").mousedown(function(){
        line=0.0;
        ang = 1.0;
        base_move_timer=setInterval("move_timer("+line+","+ang+")", 100);
	}).mouseup(function(){
        clearInterval(base_move_timer);
        input = {"line":0.0,"angu":0.0};
        jQuery.post('/teleoperation/move',input,function(data){
        });
	}); 

	//Buttons detection
	jQuery("#right").mousedown(function(){
        line=0.0;
        ang = -1.0;
        base_move_timer=setInterval("move_timer("+line+","+ang+")", 100);
	}).mouseup(function(){
        clearInterval(base_move_timer);
        input = {"line":0.0,"angu":0.0};
        jQuery.post('/teleoperation/move',input,function(data){
        });
	}); 

	//Keys detector
	jQuery(document).keydown(function (e) {
		keys[e.which] = true;
		printKeys();
	});

	jQuery(document).keyup(function (e) {
		delete keys[e.which];
		printKeys();
	});



    //Change view events
    jQuery("#radioLeft").click(function(){
        output = {image:"live_leftEye", quality: quality, width: widthCamera, height: heightCamera};
        jQuery.post('/mjpegServer/getUrlFrom',output,function(data) {

            if( cameraURL.indexOf(data) == -1 ){
                stopCmd = cameraURL.replace("stream","stop");
                jQuery.get(stopCmd);
            }

            cameraURL = data+"&t="+new Date().getTime();;
            jQuery("#iframeTeleoperation").attr("src","");
            jQuery("#iframeTeleoperation").attr("src",cameraURL);
        });
    });
    jQuery("#radioRight").click(function(){
        output = {image:"live_rightEye", quality: quality, width: widthCamera, height: heightCamera};
        jQuery.post('/mjpegServer/getUrlFrom',output,function(data) {
            if( cameraURL.indexOf(data) == -1 ){
                stopCmd = cameraURL.replace("stream","stop");
                jQuery.get(stopCmd);
            } 

            cameraURL = data+"&t="+new Date().getTime();;
            jQuery("#iframeTeleoperation").attr("src","");
            jQuery("#iframeTeleoperation").attr("src",cameraURL);
        });
    });
    jQuery("#radio3d").click(function(){
            output = {image:"live_3d", quality: quality, width: widthCamera, height: heightCamera};
            jQuery.post('/mjpegServer/getUrlFrom',output,function(data) {

                if( cameraURL.indexOf(data) == -1 ){
                    stopCmd = cameraURL.replace("stream","stop");
                    jQuery.get(stopCmd);
                } 

                cameraURL = data+"&t="+new Date().getTime();;
                jQuery("#iframeTeleoperation").attr("src","");
                jQuery("#iframeTeleoperation").attr("src",cameraURL);
           });
    });


/*
    Event to change head movement types
*/

    jQuery("#head_type1").click(function(){
            input = {"head_move_type":"1"};
            jQuery.post('/teleoperation/changeHeadMoveType',input,function(data) {
           });
    });

    jQuery("#head_type2").click(function(){
            input = {"head_move_type": "2"};
            jQuery.post('/teleoperation/changeHeadMoveType',input,function(data) {
           });
    });


/*
    Event for the camera size change
*/
    jQuery("#video_size_select").change(function() {
       
        value=jQuery("#video_size_select :selected").val();        
        jQuery("#iframeTeleoperation").attr("width",value*320);
        jQuery("#iframeTeleoperation").attr("height",value*240);
        jQuery("#imgInvisible").attr("width",value*320);
        jQuery("#imgInvisible").attr("height",value*240);  
    
        top_value = value*240+100;
        
  //      jQuery("#first_table_first_row").attr("style","height:"+value*240+"px;"); 
        jQuery("#second_table").attr("style","position:absolute;top:"+top_value+"px;width:100%;");
        jQuery("#qbo_video").attr("style","width:100%;height:"+value*240+"px;");  

        //Notify the server of the changes in video size
        input = {"width": value*320, "height":value*240};
        jQuery.post('/teleoperation/changeVideoSize',input,function(data) {
       });
	//setFlashPosition();
    });


    jQuery("#textarea")
      .focus(function() {
            if (this.value === this.defaultValue 
                || this.value === "${language['message_sent']}"
                || this.value === "${language['error_sending_message']}") {               
                jQuery("#textarea").attr("style","color:black;font-weight:normal;");
                 this.value = '';
            }
      })
      .blur(function() {
            if (this.value === '') {
                this.value = this.defaultValue;
            }
    });

/*
    Send button event
*/
    jQuery("#send_text").click(function(){
        message=jQuery("#textarea").val();
        input = {"message": message};

        jQuery("#textarea").attr("disabled", "disabled");
        jQuery("#textarea").val("${language['sending_message']}");
        jQuery.post('/teleoperation/speak',input,function(data) {
            if(data!="true")
            {    
                jQuery("#textarea").val("${language['error_sending_message']}");
                jQuery("#textarea").attr("style","color:red;font-weight:bold;");
            }
            else
            {   
              jQuery("#textarea").val("${language['message_sent']}");
                jQuery("#textarea").attr("style","color:green;font-weight:bold;");
            }
            
            jQuery("#textarea").trigger('blur');
            jQuery("#textarea").removeAttr("disabled");
       });   
    });



	// Flash position
	//setFlashPosition();
	//jQuery(window).resize(function(){ setFlashPosition(); });


	// SIP buttons
	jQuery("#btn_connect").click(function(){
		jQuery("#btn_hungup").show();
		jQuery("#btn_connect").hide();
	});
        jQuery("#btn_hungup").click(function(){
                jQuery("#btn_connect").show();
                jQuery("#btn_hungup").hide();
        });

}

function printKeys() {
	var keypressed = '';
	for (var i in keys) {
		if (!keys.hasOwnProperty(i)) continue;
		keypressed += i + ' ';
	}
          textAreaFocused = jQuery("#textarea").is(":focus");
         var d=new Date();
	 var time_now = d.getTime();
         var time_diff = time_now-last_time_pressed;


          if (keypressed.indexOf("87") != -1  && keypressed.indexOf("65") != -1 && !textAreaFocused  ) {                
                input = {"line":LINEAR_SPEED,"angu":ANGULAR_SPEED};

                if(!(movement_mode == 1 && time_diff<TIME_THRESHOLD))
	        {       
                        jQuery.post('/teleoperation/move',input,function(data){
                        });
        	          last_time_pressed = time_now;
                	  movement_mode=1;
                }
          
          }else if (keypressed.indexOf("87") != -1  && keypressed.indexOf("68") != -1  && !textAreaFocused ) {
                input = {"line":LINEAR_SPEED,"angu":ANGULAR_SPEED*-1};

                 if(!(movement_mode == 2 && time_diff<TIME_THRESHOLD))
	        {       
                	jQuery.post('/teleoperation/move',input,function(data){
	                });
                  last_time_pressed = time_now;
                  movement_mode=2;
                }
 

          }else if (keypressed.indexOf("83") != -1  && keypressed.indexOf("65") != -1 && !textAreaFocused ) {
                input = {"line":LINEAR_SPEED*-1,"angu":ANGULAR_SPEED*-1};
                  if(!(movement_mode == 3 && time_diff<TIME_THRESHOLD))
	        {       
               		jQuery.post('/teleoperation/move',input,function(data){
	                });
		 last_time_pressed = time_now;
                  movement_mode=3;
                }
 
          }else if (keypressed.indexOf("83") != -1  && keypressed.indexOf("68") != -1 && !textAreaFocused ) {
                input = {"line":LINEAR_SPEED*-1,"angu":ANGULAR_SPEED};
                 if(!(movement_mode == 4 && time_diff<TIME_THRESHOLD))
	        {       
	                jQuery.post('/teleoperation/move',input,function(data){
        	        });
                  last_time_pressed = time_now;
                  movement_mode=4;
                }
 
          }else if (keypressed.indexOf("68") != -1 && !textAreaFocused ) {
                input = {"line":0,"angu":ANGULAR_SPEED*-1};

                 if(!(movement_mode == 5 && time_diff<TIME_THRESHOLD))
	        {       
	                jQuery.post('/teleoperation/move',input,function(data){
        	        });
                	  last_time_pressed = time_now;
                  	movement_mode=5;
                }
 
          }else if (keypressed.indexOf("65") != -1  && !textAreaFocused) {
                input = {"line":0,"angu":ANGULAR_SPEED};
 
               if(!(movement_mode == 6 && time_diff<TIME_THRESHOLD))
	        {       
			jQuery.post('/teleoperation/move',input,function(data){
                	});
                   last_time_pressed = time_now;
                  movement_mode=6;
                }
          }else if (keypressed.indexOf("87") != -1 && !textAreaFocused ) {
                input = {"line":LINEAR_SPEED,"angu":0};

                if(!(movement_mode == 7 && time_diff<TIME_THRESHOLD))
	        {       
                	jQuery.post('/teleoperation/move',input,function(data){
                	});
                  last_time_pressed = time_now;
                  movement_mode=7;
                }
 
          }else if (keypressed.indexOf("83") != -1  && !textAreaFocused) {
                input = {"line":LINEAR_SPEED*-1,"angu":0};
                
                if(!(movement_mode == 8 && time_diff<TIME_THRESHOLD))
	        {       
                	jQuery.post('/teleoperation/move',input,function(data){
                	});                    
                  last_time_pressed = time_now;
                  movement_mode=8;
                }
          }else if (keypressed.indexOf("13") != -1  && textAreaFocused && !$('#textarea').attr("disabled")) {
                jQuery("#send_text").trigger('click');

          }else{

		//paramos
/*		input = {"line":0,"angu":0};
                jQuery.post('/teleoperation/move',input,function(data){
                }); 	*/
	  }      
}


//In order to set the flash position in an accurate place, just near the video
/*
function setFlashPosition(){

	videoPosition = jQuery("#iframeTeleoperation").offset();
	marginLeft = 10;

	_left = videoPosition.left + jQuery("#iframeTeleoperation").width() + marginLeft;
	_top = videoPosition.top + jQuery("#iframeTeleoperation").height() - 150 - jQuery("#flashButtons").height(); //237 is the height of the flass



	jQuery("#div_audio").offset({ top: _top, left: _left });
	

}
*/
