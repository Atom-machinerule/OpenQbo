var ipLocal="192.168.4.104";
var imgPort="8080";
var quality=50;
var widthCamera=250;
var heightCamera=250;


var defaultImg;//="http://"+ipLocal+":"+imgPort+"/snapshot?topic=/stereo/right/image_raw?quality="+quality+"?width="+widthCamera+"?height="+heightCamera;

var recognizeObjImg;// = "http://"+ipLocal+":"+imgPort+"/snapshot?topic=/qbo_stereo_selector/viewer?quality="+quality+"?width="+widthCamera+"?height="+heightCamera;

var recognizeFaceImg;// = "http://"+ipLocal+":"+imgPort+"/snapshot?topic=/qbo_face_tracking/viewer?quality="+quality+"?width="+widthCamera+"?height="+heightCamera;


var fps=24;

var ctx;
var img = new Image();
var canvas;
var loop;

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


function startEverything(){



		$.post('/mjpegServer/start', function(data){
			//Getting images
		 	output = {image:"live_rightEye", quality: quality, width: widthCamera, height: heightCamera};
	                $.post('/mjpegServer/getUrlFrom',output,function(data) {
				defaultImg = "http://"+ipLocal+":"+imgPort+data;
				cameraURL=defaultImg;
	                });

			output = {image:"objects", quality: quality, width: widthCamera, height:heightCamera};
	                $.post('/mjpegServer/getUrlFrom',output,function(data) {
	                        recognizeObjImg = "http://"+ipLocal+":"+imgPort+data;
        	        });

			output = {image:"faces", quality: quality, width: widthCamera, height:heightCamera};
	                $.post('/mjpegServer/getUrlFrom',output,function(data) {
        	                recognizeFaceImg = "http://"+ipLocal+":"+imgPort+data;
	                });
		});


/*		$( "input:submit, a, button" ).button();
		$( "#radio" ).buttonset();*/


		//Camera settings
		//Inicializacion de vriables para refreshWebCam
        	canvas = document.getElementById('canvasWebcam');
	        ctx=canvas.getContext('2d');
		loop=setInterval(refreshWebCam,fps);


		$("#training").click(function(){
			name2Learn =  $("#face_object_name").val().toUpperCase();		
			if(name2Learn==""){
				$("#answer").html(${language['test']});
			}else{

				 //launch Nodes
        	                $.post('/training/launchNodes',function(data) {	
					       if(objectORface == "object"){
	                		           cameraURL=recognizeObjImg;
        	        		       }else{
		        	                   cameraURL=recognizeFaceImg;
        		        	       }
	        		               auxTime4Coundown = new Date().getTime();
	                		       action="learning";
        		        	       countdown=3;
			                       bool_drawing=true;
					
				});
			}
		});

                $("#guessing").click(function(){
			//launch Nodes
			$.post('/training/launchNodes',function(data) {
				if(objectORface == "object"){
					cameraURL=recognizeObjImg;
				}else{
					cameraURL=recognizeFaceImg;
				}
                                auxTime4Coundown = new Date().getTime();
                                action="recognizing";
                                countdown=3;
                                bool_drawing=true;
                        });
		});

		$("#radioFace").click(function(){
			$.post('/training/selectFaceRecognition',function(data) {
				$("#personORobject").html("person");
			});	
		});

                $("#radioObject").click(function(){
                        $.post('/training/selectObjectRecognition',function(data) {
				$("#personORobject").html("object");
			});
                });
		
};













function refreshWebCam(){

	//We check wether we are in the tab that is showing the canvas y we just left, so we have to stop everthing
	if( $("#ui-tabs-4").hasClass('ui-tabs-hide') ){
		clearInterval(loop);

		//stop mjpeg server
                $.post('/mjpegServer/stop', function(data) {
	               if(data=="ERROR"){
                       }
                });
	}


	try{
        img.src=cameraURL;

        img.onload = function(){
                ctx.drawImage(img, 0, 0, img.width, img.height);
                if(bool_drawing){
                        //Painting countdown
                        if(countdown > -1){

                                if ( new Date().getTime() - auxTime4Coundown >= 1000 ){
                                        countdown = countdown - 1;
                                        auxTime4Coundown = new Date().getTime();
                                }
                                if(countdown != -1){
                                        ctx.font = "bold 36px sans-serif";
                                        ctx.fillStyle = "rgb(255, 0, 0)";
                                        ctx.fillText(countdown.toString(), 10, 200);
                                }else{
                                        countdown = -100;
                                        //alert("lanzadera");
                                        //Lanzamos la grabacion de fotogramas                           
                                        //ID_grabaFotogramas = setInterval(grabaFotogramas, delayBtwFotogramsCaptured);

                                        if(action=="learning"){
                                                recording=true;
						disableRadioBotton(true);
                                                startLearning();
                                        }else if(action=="recognizing"){
                                                watching=true;
						disableRadioBotton(true);
                                                startRecognition();
                                        }

                                }

                        }

                        if(recording && countdown==-100){
                                ctx.font = "bold 36px sans-serif";
                                ctx.fillStyle = "rgb(255, 0, 0)";
                                ctx.fillText(msgWebCamRec, 10, 200);
                        }else if(training && countdown==-100){
			        ctx.font = "bold 36px sans-serif";
                                ctx.fillStyle = "rgb(255, 0, 0)";
                                ctx.fillText(msgWebCamTraining, 10, 200);
                        }else if(watching && countdown==-100){
                                ctx.font = "bold 36px sans-serif";
                                ctx.fillStyle = "rgb(255, 0, 0)";
                                ctx.fillText(msgWebCamWatching, 10, 200);
                        }

                }
        }
	}catch(e){
	}

}


function startLearning(){
                                //lanzamos aprendizaje
                                output = { objectName : name2Learn };
                                $.post('/training/startLearning' ,output, function(data) {
                                        recording=false;
                                        training=true;
                                        $.post('/training/startTraining' , function(data) {

                                        training=false;
                                        bool_drawing=false;
				                        alert(data);
                                        if(data){
					                        $("#answer").html("texto de se ha reconocido bien "+name2Learn);
                                        }else{//error
					                        $("#answer").html("texto de errorno se ha podido reoconcer  "+name2Learn);
                                        }
                                        cameraURL=defaultImg;
                                        $.post('/training/stopNode',function(data) {
                                        });
                                });
                        });
}


function startRecognition(){
	 //start recognition
                                $.post('/training/startRecognition',function(data) {
                                        cameraURL=defaultImg;
                                        watching=false;

                                        if(data!=""){
                                                $("#answer").html(data.toLowerCase());
                                        }else{
                                                $("#answer").html("No lo se");
                                        }

                                        //paramos nodo
                                        $.post('/training/stopNode',function(data) {
                                                    alert("Hemos matado los nodos");
                                        });


                                        cameraURL=defaultImg;
					disableRadioBotton(false);

                                })
                                .error(function() {


                                        //paramos nodo
                                        $.post('training/stopNode',function(data) {
                                        });

                                        cameraURL=defaultImg;
				        disableRadioBotton(false);
                                });
}









function disableRadioBotton(disable){

	if(disable){
		$("#radio").attr("disabled", "disabled");
	}else{
		$("#radio").removeAttr("disabled");
	}


	
}
