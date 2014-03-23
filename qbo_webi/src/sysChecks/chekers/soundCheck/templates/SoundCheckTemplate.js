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

var listCheck=new Array();

function startEverything(){



    jQuery("#soundCheckImg").attr("src","sysChecks/static/img/loading.gif");

    jQuery("#leftAudioButton").click(function(data){       

        jQuery("#stopLeftAudioButton").show();
        jQuery("#leftAudioButton").hide();

	playLeftAudio();

    });





    jQuery("#rightAudioButton").click(function(data){

        jQuery("#stopRightAudioButton").show();
        jQuery("#rightAudioButton").hide();

	playRightAudio();
    });


    jQuery("#centerAudioButton").click(function(data){

        jQuery("#stopCenterAudioButton").show();
        jQuery("#centerAudioButton").hide();

	playBothAudio();
    });



    // Yes buttons
    jQuery("#yesButtonLeft").click(function(data){
        listCheck['left'] = true;
        jQuery("#checkLeft").show();
        finalCheck();
        jQuery("#leftOk").show();
        jQuery("#leftWrong").hide();
    });

    jQuery("#yesButtonRight").click(function(data){
        listCheck['right']=true;
        jQuery("#checkRight").show();
        finalCheck();
        jQuery("#rightOk").show();
        jQuery("#rightWrong").hide();
    });

    jQuery("#yesButtonCenter").click(function(data){
        listCheck['center']=true;
        jQuery("#checkCenter").show();
        finalCheck();
        jQuery("#centerOk").show();
        jQuery("#centerWrong").hide();
    });


    // No buttons
    jQuery("#noButtonLeft").click(function(data){
        listCheck['left'] = false;
        jQuery("#checkLeft").show();
        finalCheck();
        jQuery("#leftOk").hide();
        jQuery("#leftWrong").show();
    });

    jQuery("#noButtonRight").click(function(data){
        listCheck['right']=false;
        jQuery("#checkRight").show();
        finalCheck();
        jQuery("#rightOk").hide();
        jQuery("#rightWrong").show();
    });

    jQuery("#noButtonCenter").click(function(data){
        listCheck['center']=false;
        jQuery("#checkCenter").show();
        finalCheck();
        jQuery("#centerOk").hide();
        jQuery("#centerWrong").show();
    });










}


//We check if the three types of sound are correct or not
function finalCheck(){

    check = false;
    try{
        check = listCheck['left'] && listCheck['right'] && listCheck['center'] ;

        if (check == true){
           jQuery("#soundCheckImg").attr("src","sysChecks/static/img/ok.png");
           window.soundCheckImgSrc="sysChecks/static/img/ok.png";
        }else if(check == false){ //the other option was undefined
           jQuery("#soundCheckImg").attr("src","sysChecks/static/img/wrong.png");  
           window.soundCheckImgSrc="sysChecks/static/img/wrong.png";
        }
    }catch(e){
        check = false;        
    }

}


function playLeftAudio(){
        jQuery.post('/checkers/index/soundCheck/play/left',function(data) {
            jQuery("#stopLeftAudioButton").hide();
            jQuery("#leftAudioButton").show();
            jQuery("#menuLeftAudio").show();
        });

}


function playRightAudio(){
        jQuery.post('/checkers/index/soundCheck/play/right',function(data) {
            jQuery("#stopRightAudioButton").hide();
            jQuery("#rightAudioButton").show();
            jQuery("#menuRightAudio").show();
        });

}


function playBothAudio(){
        jQuery.post('/checkers/index/soundCheck/play/center',function(data) {
            jQuery("#stopCenterAudioButton").hide();
            jQuery("#centerAudioButton").show();
            jQuery("#menuCenterAudio").show();
        });
}



