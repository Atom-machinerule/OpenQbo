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
var actualUrlImg="";
var MAX_NUM_TRIES = 10; // after 10 consecutive errors we will stop showing images
var loop;
var imgPort="8081";

var quality=50;
var widthCamera=320;
var heightCamera=240;


var defaultImg;//="http://"+ipLocal+":"+imgPort+"/stream?topic=/stereo/right/image_raw?quality="+quality+"?width="+widthCamera+"?height="+heightCamera;

var recognizeObjImg;// = "http://"+ipLocal+":"+imgPort+"/stream?topic=/qbo_stereo_selector/viewer?quality="+quality+"?width="+widthCamera+"?height="+heightCamera;

var recognizeFaceImg;// = "http://"+ipLocal+":"+imgPort+"/stream?topic=/qbo_face_tracking/viewer?quality="+quality+"?width="+widthCamera+"?height="+heightCamera;


var fps=24;

var ctx;
var img = new Image();
var canvas;

var ctxQboVision;
var canvasQboVision;

var action;
var countdown;
var bool_drawing=false;
var recording = false;
var watching = false;
var sendingToCloud = false;
var training = false;
var auxTime4Coundown;

var name2Learn;

var objectORface="object";
var first_itr="true";

var msgWebCamWatching = "Watching";
var msgWebCamTraining = "Training...";
var msgSendingToCloud = "Sending to Q.bo Cloud...";
var msgWebCam = "";
var msgWebCamRec = "Rec";

var timeout4Message;

function startEverything(){

        //Camera settings
        //Inicializacion de vriables para refreshWebCam
        canvas = document.getElementById('canvasWebcam');
        ctx=canvas.getContext('2d');

		//Getting images
		output = {image:"live_leftEye", quality: quality, width: widthCamera, height: heightCamera};
	    jQuery.post('/mjpegServer/getUrlFrom',output,function(data) {
				//getDefaultImg = "http://"+ipLocal+":"+imgPort+data;
				defaultImg = data;
	    });

		output = {image:"live_objects", quality: quality, width: widthCamera, height:heightCamera};
	    jQuery.post('/mjpegServer/getUrlFrom',output,function(data) {
	            //getRecognizeObjImg = "http://"+ipLocal+":"+imgPort+data;
	            recognizeObjImg = data;
                launchNodes("object");
        });

		output = {image:"live_faces", quality: quality, width: widthCamera, height:heightCamera};
	    jQuery.post('/mjpegServer/getUrlFrom',output,function(data) {
                recognizeFaceImg = data;
	    });


        jQuery("#training").click(function(){
            jQuery("#inputFaceObjectName").show();
        });

		jQuery("#ok_start_training").click(function(){
			name2Learn =  jQuery("#face_object_name").val().toUpperCase();		
			if(name2Learn==""){
                showMessage("${language['error_no_name_written']}");
			}else{

                   auxTime4Coundown = new Date().getTime();
                   action="learning";
                   countdown=3;
                   bool_drawing=true;
                   loop=setInterval('drawInfoinCanvas();',1000);
			}
		});

        jQuery("#guessing").click(function(){
                auxTime4Coundown = new Date().getTime();
                action="recognizing";
                countdown=3;
                bool_drawing=true;
                loop=setInterval('drawInfoinCanvas()',1000); 
            });

		jQuery("#radioFace").click(function(){
            launchNodes("face");
		});	

		jQuery("#radioObject").click(function(){
            launchNodes("object");
		});	
}

function drawInfoinCanvas(){
        try{
            if(bool_drawing){
                        //Painting countdown
                        if(countdown > -1){

                                if ( new Date().getTime() - auxTime4Coundown >= 1000 ){
                                        countdown = countdown - 1;
                                        auxTime4Coundown = new Date().getTime();
                                }
                                if(countdown != -1){
                                        ctx.clearRect ( 0 , 0 , canvas.width , canvas.height );
                                        ctx.font = "bold 36px sans-serif";
                                        ctx.fillStyle = "rgb(255, 0, 0)";
                                        ctx.fillText(countdown.toString(), 10, 200);

                                }else{
                                        countdown = -100;
                                        //Lanzamos la grabacion de fotogramas                           
                                        //ID_grabaFotogramas = setInterval(grabaFotogramas, delayBtwFotogramsCaptured);

                                        if(action=="learning"){
                                                recording=true;
                                                disableRadioBotton(true);
                                                startLearningAndTraining();
                                        }else if(action=="recognizing"){
                                                watching=true;
                                                disableRadioBotton(true);
                                                startRecognition();
                                        }
                                }
                        }
                        if(recording && countdown==-100){
                                ctx.clearRect ( 0 , 0 , canvas.width , canvas.height );
                                ctx.font = "bold 36px sans-serif";
                                ctx.fillStyle = "rgb(255, 0, 0)";
                                ctx.fillText(msgWebCamRec, 10, 200);
                        }else if(training && countdown==-100){
                                ctx.clearRect ( 0 , 0 , canvas.width , canvas.height );
                                ctx.font = "bold 36px sans-serif";
                                ctx.fillStyle = "rgb(255, 0, 0)";
                                ctx.fillText(msgWebCamTraining, 10, 200);
                        }else if(sendingToCloud && countdown==-100){
                                ctx.clearRect ( 0 , 0 , canvas.width , canvas.height );
                                ctx.font = "bold 20px sans-serif";
                                ctx.fillStyle = "rgb(255, 0, 0)";
                                ctx.fillText(msgSendingToCloud, 10, 200);
                        }else if(watching && countdown==-100){
                                ctx.clearRect ( 0 , 0 , canvas.width , canvas.height );
                                ctx.font = "bold 36px sans-serif";
                                ctx.fillStyle = "rgb(255, 0, 0)";
                                ctx.fillText(msgWebCamWatching, 10, 200);
                        }

            }else{
                ctx.clearRect ( 0 , 0 , canvas.width , canvas.height );
                clearInterval(loop);
            }

        }catch(e){
            ctx.clearRect ( 0 , 0 , canvas.width , canvas.height );
            clearInterval(loop);
        }
}

function startLearningAndTraining(){
    //lanzamos aprendizaje
    output = { objectName : name2Learn };

    var cloudIsChecked = jQuery("#checkboxCloud").attr('checked')?true:false;

    if(cloudIsChecked && objectORface == "object") //Share with the Cloud
    {
        jQuery.post('/training/startLearning' ,output, function(data) {
            recording=false;
            sendingToCloud = true;
            jQuery.post('/training/sendToCloud',function(data) {
                sendingToCloud = false;
                training=true;
                jQuery.post('/training/startTraining' , function(data) {
                    training=false;
                    bool_drawing=false;

                    if(data){
                        showMessage("${language['learning_ok']}"+name2Learn+"${language['share_it']}");
                    }else{//error
                        showMessage("${language['learning_ko']}"+name2Learn); 
                    }

                    jQuery("#inputFaceObjectName").hide();
               });
             });
        });
    }    
    else //Do NOT share with the Cloud
    {
        jQuery.post('/training/startLearning' ,output, function(data) {
            recording=false;
            training=true;
            jQuery.post('/training/startTraining' , function(data) {
                training=false;
                bool_drawing=false;

                if(data){
                    showMessage("${language['learning_ok']}"+name2Learn);
                }else{//error
                    showMessage("${language['learning_ko']}"+name2Learn); 
                }

                jQuery("#inputFaceObjectName").hide();

            });
        });
    }
}

/*
function startRecognition(){
	 //start recognition
    
   var cloudIsChecked = jQuery("#checkboxCloud").attr('checked')?true:false;

    if(cloudIsChecked && objectORface == "object") //User Cloud Data base
    {
        alert("hola");
        jQuery.getJSON('/training/startCloudRecognition',function(data) 
        {   
            //actualUrlImg=getDefaultImg();
            watching=false;
            bool_drawing=false;
           
            //alert(data{'status'});
            if(data!=""){
                showMessage("${language['this_is_a']}"+data.toLowerCase());
            }else{
                showMessage("${language['dontKnow']}");
            }
            jQuery("#inputFaceObjectName").hide();

            disableRadioBotton(false);

        })
        
 
    }
    else //Use local Object of Face data base
    { 
        jQuery.post('/training/startRecognition',function(data) {
            //actualUrlImg=getDefaultImg();
            watching=false;
            bool_drawing=false;

            if(data!=""){
                showMessage("${language['this_is_a']}"+data.toLowerCase());
            }else{
                showMessage("${language['dontKnow']}");
            }

            jQuery("#inputFaceObjectName").hide();

            disableRadioBotton(false);

        })
    }


    .error(function() {
        //paramos nodo
        disableRadioBotton(false);
    });
}
*/



function startRecognition(){
 

    var cloudIsChecked = jQuery("#checkboxCloud").attr('checked')?true:false;

    if(cloudIsChecked && objectORface == "object") //User Cloud Data base
    {
        jQuery.getJSON('/training/startCloudRecognition',function(data) 
        {   
            //actualUrlImg=getDefaultImg();
            watching=false;
            bool_drawing=false;
            
            if(data['status']=="ok"){
                if(data['object'] != "")
                  showMessage("${language['this_is_a']}"+data['object'].toLowerCase());
                else
                  showMessage("${language['dontKnow']}");
            }else{
                showMessage("PROBLEM: "+data['status']);
            }
            jQuery("#inputFaceObjectName").hide();
    
            disableRadioBotton(false);
        })
    }
    else
    {

      //start recognition
      jQuery.post('/training/startRecognition',function(data) {
          actualUrlImg=getDefaultImg();
          watching=false;
          bool_drawing=false;
  
          if(data!=""){
              showMessage("${language['this_is_a']}"+data.toLowerCase());
          }else{
              showMessage("${language['dontKnow']}");
          }
  
          jQuery("#inputFaceObjectName").hide();
  
          disableRadioBotton(false);
  
     })
      .error(function() {
          //paramos nodo
          disableRadioBotton(false);
      });
    }
  }

function disableRadioBotton(disable){

	if(disable){
	jQuery("#radio").attr("disabled", "disabled");
	}else{
		jQuery("#radio").removeAttr("disabled");
	}	
}

function showMessage(text){
    try{
        cleatTimeout(timeout4Message);
    }catch(e){}

    jQuery("#divQboMessage").show();    

    //We set the position for the error "dialog", where an avatar of qbo appears and says something
    positionCanvas = jQuery("#divQboVision").offset();

    //jQuery("#divQboMessage").offset({ top: positionCanvas.top+jQuery("#divQboVision").height()-jQuery("#qboAvatar").height(),
      //                                left: positionCanvas.left+jQuery("#divQboVision").width()+20   });

    jQuery("#divQboMessage").hide();
    jQuery("#divQboMessage").fadeIn();

    jQuery("#errorText").html(text)

    timeout4Message = setTimeout('jQuery("#divQboMessage").fadeOut()',3000);
}

function getRecognizeObjImg(){
    t=new Date().getTime();
    return recognizeObjImg+"&t="+t;
}
function getRecognizeFaceImg(){
    t=new Date().getTime();
    return recognizeFaceImg+"&t="+t;
}
function getDefaultImg(){
    t=new Date().getTime();
    return defaultImg+"&t="+t;
}

function launchNodes(faceObjectString)
{
    if(faceObjectString == "face") //Face Recognition Mode
    {
        faceOnBool = "true";
        personOrObjectString = "person";
         
        //Stop the stream of the previous selected image
        //if( actualUrlImg.indexOf(recognizeFaceImg) != -1 ){

        if(!first_itr)
        {    stopCmd =  jQuery("#qboVision").attr("src").replace("stream","stop");
            jQuery.get(stopCmd);
        }
        else
        {

            first_itr = false;
        }

         
        //}   
        
        //Activate face tracking image
        jQuery("#qboVision").attr("src",getRecognizeFaceImg());
        actualUrlImg=getRecognizeFaceImg();
        jQuery("#divShareToCloud").hide();

    }   
    else //Object Recognition Mode
    {
        faceOnBool = "false";
        personOrObjectString = "object";

        if(!first_itr)
        {    stopCmd =  jQuery("#qboVision").attr("src").replace("stream","stop");
            jQuery.get(stopCmd);
        }
        else
        {

            first_itr = false;
        }


        //Activate stereo selector image
        jQuery("#qboVision").attr("src",getRecognizeObjImg());
        actualUrlImg=getRecognizeObjImg(); 
        
        jQuery("#divShareToCloud").show();
    
    }

    //Stop the nodes of the previous state
    jQuery.post('training/stopNode',function(data) { 
        args = { faceOn : faceOnBool };
        jQuery.post('/training/launchNodes',args, function(data) {
            jQuery("#personORobject").html(personOrObjectString);
            objectORface = faceObjectString;
        });
     });

}

