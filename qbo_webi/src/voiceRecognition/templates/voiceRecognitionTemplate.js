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

var oldText="";
var text="";

function startEverything(){


    //We have to leave this line in orther to make the iframe works
    jQuery('.rte-zone').rte("css url", "toolbox images url");


    jQuery("#new_lang").change(function() {

        input = {"lang":jQuery(this).val()};
        jQuery.post("/voiceRecognition/getModels",input,function(data){

            array = data.split("::");

            list="<option value='' class='ui-button ui-widget ui-state-default ui-corner-all ui-button-text-only' style='display:block;'>${language['choose_model']}</option>";
            for(i=0;i<array.length;i++){
                list = list+"<option value='"+array[i]+"' class='ui-button ui-widget ui-state-default ui-corner-all ui-button-text-only' style='display:block;'>"+array[i]+"</option>";
            }

            jQuery("#languageModel").html(list);

        });

    });


    jQuery("#languageModel").change(function() {

        jQuery("#test2").attr('disabled', 'disabled');
        jQuery("#save").attr('disabled', 'disabled');
        jQuery("#outputConsole").html("");

        input = {"lang":jQuery("#new_lang").val(),"model":jQuery(this).val()};
        jQuery.post("/voiceRecognition/getFile",input,function(data){

            if(jQuery("#languageModel").val() != ""){
                jQuery("#divTextArea").show();
                jQuery("#buttons_voiceRecog").show();
               
                jQuery("iframe").contents().find("body").html("");
 
                //jQuery("#textArea").html("");
//                arrayWords = data.split("\n");

//                for(i=0;i<arrayWords.length;i++){            
//                    jQuery("iframe").contents().find("body").append(arrayWords[i]+"</br>");
//                }


        data = data.replace(/\n/g,"</br>");
        jQuery("iframe").contents().find("body").append(data);


            }else{
                jQuery("#divTextArea").hide();
                jQuery("#buttons_voiceRecog").hide();
            }
        });
    });


    //if text area is edited, we need to pass all the test again
    jQuery("iframe").contents().find("body").keydown(function(e){
        jQuery("#test2").attr('disabled', 'disabled');
        jQuery("#save").attr('disabled', 'disabled');        
    });
    

    jQuery("#test1").click(function(data){
        jQuery("#test2").attr('disabled', 'disabled');
        jQuery("#save").attr('disabled', 'disabled');
        sendAndCheck(1);
    });

    jQuery("#test2").click(function(data){
        jQuery("#save").attr('disabled', 'disabled');
        sendAndCheck(2);
    });

    jQuery("#save").click(function(data){
        jQuery('*').css('cursor', 'progress');
        input = {"text":text,"lang":jQuery("#new_lang").val(),"model":jQuery("#languageModel").val()};
        jQuery.post("/voiceRecognition/saveToFile",input,function(data){
           jQuery('*').css('cursor', '');
           jQuery("#outputConsole").html("${language['saved_ok']}"); 
        });
    });



    //Acustic Model stuff
    jQuery("#stop_btn").hide();
    jQuery("#play_off").show();
    jQuery("#play").hide();
    jQuery("#save_off").show();
    jQuery("#save_am").hide();

    jQuery("#save_btn").attr('disabled', 'disabled');
    jQuery("#play_btn").attr('disabled', 'disabled');
// jQuery("#test2").removeAttr("disabled");



//Eventos AUDIO

    jQuery("#rec_btn").click(function() {
            jQuery("#stop_btn").show();
            jQuery("#rec_btn").hide();

            jQuery("#play_off").show();
            jQuery("#play").hide();
            jQuery("#play_btn").attr('disabled', 'disabled');

            jQuery("#save_off").show();
            jQuery("#save_am").hide();
            jQuery("#save_btn").attr('disabled', 'disabled');

            botonesOn=false;
       
            jQuery.post('/voiceRecognition/rec', function(data) {
                if(data=="ERROR"){
                    //window.location = "http://"+ipBasic+"/brainchat_sp/";
                    printMessage("${language['voiceRecog_error_connect_server']}","error");

                }
            });
    });



    jQuery("#stop_btn").click(function() {


        jQuery("#stop_btn").hide();
        jQuery("#rec_btn").show();

       
        jQuery('*').css('cursor', 'progress');
 
        
        jQuery.post('/voiceRecognition/stop',function(data) {

            if (data=="error"){
                printMessage("${language['voiceRecog_error_connect_server']}","error");
            }
       

 
            jQuery('*').css('cursor', '');
    
            jQuery("#play_off").hide();
            jQuery("#play").show();
            jQuery("#play_btn").removeAttr("disabled");
    

            jQuery("#save_off").hide();
            jQuery("#save_am").show();
            jQuery("#save_btn").removeAttr("disabled");        

            jQuery("#Input").val(data);

            //alert(data+" ");

            botonesOn=true;
            
        });

    });

    jQuery("#play_btn").click(function() {
        if(botonesOn){
         jQuery.post('/voiceRecognition/play', function(data) {              
            //jQuery("#title").html(data) ;

         });
        }
    });

    jQuery("#save_btn").click(function() {
        if(botonesOn){
            jQuery('*').css('cursor', 'progress');

            value = jQuery("#Input").val();

            //alert(value);

            value = value.replace("'","\\'");


            //alert(value);

            var param = { transcripcion: value };

            jQuery.post('/voiceRecognition/save',param, function(data) {                


                if (data=="error"){
                    printMessage("${language['voiceRecog_error_connect_server']}","error");
                }else{
                    printMessage("${language['voiceRecog_response_from_server_ok']}","info");
                }

                jQuery('*').css('cursor', '');
                //antes sacabamos por pantalla los tiempos totales, pero ya nop $("#Input").val(data);
                jQuery("#play_off").show();
                jQuery("#play").hide();
                jQuery("#play_btn").attr('disabled', 'disabled');


                jQuery("#save_off").show();
                jQuery("#save_am").hide();
                jQuery("#save_btn").attr('disabled', 'disabled');  
            });
        }
    });





}


function printMessage(string,nature){    
    if (nature=="info"){
        jQuery("#info_am").css('color', 'blue');    
    }else if (nature=="error"){
        jQuery("#info_am").css('color', 'red'); 
    }

    jQuery("#info_am").html(string);
    jQuery("#info_am").fadeIn("slow");
    setTimeout('jQuery("#info_am").fadeOut("slow",function(){ jQuery("#info_am").html("");  });',1500);
}




function sendAndCheck(numTest){
        //We get the text
        a = jQuery("iframe").contents().find("body").html();
        a = a.replace(/<span style\="color\:black;">/g,"");
        a = a.replace(/<span style="color:red;">/g,"");
        a = a.replace(/<\/span>/g,"");
        a = a.replace(/<div><br><\/div>/g,"<br>");
        a = a.replace(/<div>/g,"<br>");

        a = a.replace(
            new RegExp( "\\s*<br>\\s*", "g" ),
            "<br>");

        a = a.replace(
            new RegExp( "(<br>)+", "g" ),
            "<br>");


        a = a.replace(/<br>\[/g,"\n\n[");
        a = a.replace(/<br>/g,"\n");


        a = a.replace(/<\/div>/g,"");
        a = a.replace(/&nbsp;/g," ");
        a = a.replace(/<font color="#ff0000">/g,"");
        a = a.replace(/<font color="#000000">/g,"");
        a = a.replace(/<\/font>/g,"");


        oldText = a;

        jQuery('*').css('cursor', 'progress');

        input = {"text":oldText,"lang":jQuery("#new_lang").val()};
        jQuery.post("/voiceRecognition/test"+numTest,input,function(data){
        jQuery('*').css('cursor', '');

            //data will have a list of words not allowed
            if(data == ""){
                //success
                jQuery("iframe").contents().find("body").html("");

                arrayContent = oldText.split("\n");
                for(i=0;i<arrayContent.length;i++){
                    jQuery("iframe").contents().find("body").append(arrayContent[i]+"</br>");
                }


                if(numTest==1){
                    jQuery("#test2").removeAttr("disabled");
                    jQuery("#outputConsole").html("${language['test1_ok']}");
                }else if(numTest==2){
                    jQuery("#save").removeAttr("disabled");
                    jQuery("#outputConsole").html("${language['test2_ok']}");
                }

            }else{
                if(numTest==1){
                    jQuery("#outputConsole").html("${language['test1_wrong']}");
                }else if(numTest==2){
                    jQuery("#outputConsole").html("${language['test2_wrong']}");
                }


                //fail
                jQuery("iframe").contents().find("body").html("");
                arrayContent = oldText.split("\n");

                arrayError = data.split("::");
                for(i=0;i<arrayContent.length;i++){
                    arrayWords = arrayContent[i].split(" ");
                    line="";
                    for(j=0;j<arrayWords.length;j++){

                        if( arrayError.indexOf(arrayWords[j].toUpperCase()) != -1  ){
                            line=line+"<span style='color:red;'>"+arrayWords[j]+"</span> ";
                        }else{
                            line=line+"<span style='color:black;'>"+arrayWords[j]+"</span> ";
                        }
                    }
                    jQuery("iframe").contents().find("body").append(line+"</br>");
                }

            }
            text=oldText;
            oldText="";
        });

}




