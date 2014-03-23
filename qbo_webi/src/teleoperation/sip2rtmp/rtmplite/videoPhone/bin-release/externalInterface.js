function flash(flashLabel) {
    if (navigator.appName.indexOf("Microsoft") != -1) {
        return window[flashLabel];
    } else {
        return document[flashLabel];
    }
}

var qbobotUserAuth;

function call(){



    jQuery.getJSON("/teleoperation/getAuthBot/",function(data){

//                        jQuery("#btn_call").attr("onClick","call('sip:"+data.auth+"@localhost')");

                        qbobotUserAuth = data.auth;
  //                      call('sip:'+data.auth+'@localhost');

	flash("VideoPhone").callRobot('sip:'+data.auth+'@localhost');
    });


}


function hangUp(){

    jQuery('*').css('cursor', 'progress');
    flash("VideoPhone").hangUpRobot();
    setTimeout('flash("VideoPhone").disconnect();  jQuery.post("/teleoperation/stopSIPService",function(data){ jQuery("*").css("cursor", "");  });  ',1000);

}

function connect(gatewayURL, sipURL, authName, authPass, displayName){

    jQuery.post("/teleoperation/startSIPService",function(data){	


        //Authenticacion for SIP server
        jQuery.getJSON("/teleoperation/getAuth/",function(data){

		gatewayURL = 'rtmp://'+data.host+'/sip';
		sipURL = 'sip:'+data.auth+'@localhost';
		authName = data.auth;
		authPass = 'webi';
		displayName = 'webi';

		flash("VideoPhone").connectRobot(gatewayURL, sipURL, authName, authPass, displayName);

		call();

	});
    });

}
function disconnect(){
   // flash("VideoPhone").disconnect();
}



