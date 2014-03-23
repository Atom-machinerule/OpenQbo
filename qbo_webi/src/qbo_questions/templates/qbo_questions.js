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
 * Authors: Miguel Julian <miguel.julian@openqbo.com>; 
 *          Daniel Cuadrado <daniel.cuadrado@openqbo.com>;
 *          Sergio Merino <s.merino@openqbo.com>;
 *          Arturo Bajuelos <arturo@openqbo.com>;
*/


var lang = "${language['current_language']}"

function startEverything(){

    //We take questions from the grammar 
    input = {"lang":lang,"model":"questions"};
    jQuery.post("/voiceRecognition/getFile",input,function(data){
        questions_list = data.split("\n");

        stop_at_next_tag = false; //We only want the sentences after [senteces], when the next tag comes, i.e. [names], we skip it
        for(i in questions_list){
            if( questions_list[i] == "[sentences]" ){
                stop_at_next_tag = true;
            }else if ( stop_at_next_tag ){

                if(questions_list[i][0] == "[" || questions_list[i]==""){
                    break;
                }else{            
                    jQuery("#listQuestions").append('<option  class="ui-button ui-widget ui-state-default ui-corner-all ui-button-text-only" style="display: block;" >'+questions_list[i]+'</option>');
                }
            } 
        }
    });


    input = {"lang":lang};
    jQuery.getJSON("/qbo_questions/getActualDialogue",input, function(data){
        fillTable(data);
    });

   


    jQuery("#addSentence").click(function(data){

        answer = jQuery("#answer").val().replace('"','\\"');

        param = {"lang":lang, "question": jQuery("#listQuestions").val() , "answer": answer };
        jQuery.getJSON("/qbo_questions/addSentence",param,function(data){
           fillTable(data);
        });
    });


    jQuery(".speaker_icon").click(function(data){

        answer = jQuery("#answer").val()

        param = {"answer": answer };
        jQuery.getJSON("/qbo_questions/playSentence",param,function(data){
        });
    });


}



function fillTable(data){
     html = "";
        for (var key in data) {


            html = html + ' <tr> <td class="td_qbo_questions"> '+ key +'<a target="_blank" class="tooltip" title="Remove: '+key+'" > <img id="'+key+':::" alt="Delete '+key+'"  class="delete_sentence" src="/qbo_questions/static/img/close.png"></img></a>  </td> <td> <ul class="ul_qbo_questions"> ';

            for (i=0;i<data[key].length;i++){

                aux_id = key.replace(/"/g,'&quot;')+":::"+data[key][i].replace(/"/g,'&quot;'); 

                

                html = html + '<li>'+data[key][i]+'<a target="_blank" class="tooltip" title="Remove: '+data[key][i]+'" > <img id="'+aux_id+'" alt="Delete '+data[key][i]+'"  class="delete_sentence" src="/qbo_questions/static/img/close.png"></img> </a>  </li>';
            }


            html = html+" </ul> </td> /tr> <hr />";


       }

        jQuery(".tooltip").tooltip();

        jQuery("#dialogue_table").html(html);


    //We define the delete sentence button behaviour
    jQuery(".delete_sentence").click(function(data){
        param = {"lang":lang,"sentence2delete": jQuery(this).attr("id") };
        jQuery.getJSON("/qbo_questions/deleteSentence",param,function(data){
           fillTable(data);
        });
    });


}

