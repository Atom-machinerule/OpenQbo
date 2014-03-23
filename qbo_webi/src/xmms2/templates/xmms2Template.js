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
var play_list = {};
var pos_inicial = -1;
var pos_final = -1;
var id_moving = -1; //the id of the element which it is been moving
var id_actual_song = -1;
var loop;

function startEverything(){

    //Buttons
    jQuery("#playpause").click(function(data){
        jQuery.post("xmms2/playpause",function(data){
        if (data==1){
            jQuery("#playpause").attr("src","/xmms2/static/img/pause.png");
        }
        else{
           jQuery("#playpause").attr("src","/xmms2/static/img/play.png");
        }
        });
    });
    jQuery("#stop").click(function(data){
        jQuery("#playpause").attr("value","play");
        jQuery.post("xmms2/stop",function(data){
            jQuery(".play_icon").hide();
        });
        jQuery("#playpause").attr("src","/xmms2/static/img/play.png");
    });
    jQuery("#nxt").click(function(data){
        jQuery.post("xmms2/next",function(data){
            jQuery(".play_icon").hide();
            if(data!=-1){
                 jQuery("#play_icon_"+data).show();
            }

        });
    });
    jQuery("#prev").click(function(data){
        jQuery.post("xmms2/previous",function(data){
            jQuery(".play_icon").hide();
            if(data!=-1){
                 jQuery("#play_icon_"+data).show();
            }
        });
    });

    jQuery("#more").click(function(data){
        input = jQuery("#volume").slider("value");
        if (input<100){
            jQuery("#volume").slider("value",input+5);
            toSend={"volume":input+5};
            jQuery.post("xmms2/setVolume",toSend,function(data){});
        }
    });

    jQuery("#less").click(function(data){
        input = jQuery("#volume").slider("value");
        if (input>0){
            jQuery("#volume").slider("value",input-5);
            toSend={"volume":input-5};
            jQuery.post("xmms2/setVolume",toSend,function(data){});
        }
    });



    //Every 5 secons we check which song is playing
    loop = setInterval("scheduled()",1000);

    //Volume bar
    jQuery( "#volume" ).slider({
            value:0,
            min: 0,
            max: 100,
            step: 5,
/*            slide: function( event, ui ) {
                input = {"volume":ui.value};
                jQuery.post("xmms2/setVolume",input,function(data){
                });
            }*/
    });

    jQuery.post("xmms2/getVolume",function(data){
        jQuery("#volume").slider("value",data); 
    });

    jQuery.post("xmms2/getStatus",function(data){
        if (data==1){
            jQuery("#playpause").attr("src","/xmms2/static/img/pause.png");
        }
    });


    //Fill the playlist
    jQuery.getJSON('xmms2/getActivePlaylistSongs', function(data) {   
        fillList(data); 
    });

    jQuery( "#music_list" ).sortable();
    //jQuery( "#music_list" ).disableSelection();

    jQuery( "#music_list" ).bind( "sortstart", function(event, ui) {

        id_moving=ui.helper.attr("id");

        play_list = jQuery("#music_list").sortable('toArray');

        pos_inicial=getPosInPlayList(id_moving);        

    });

    jQuery( "#music_list" ).bind( "sortstop", function(event, ui) {
        play_list = jQuery("#music_list").sortable('toArray');

        pos_final = getPosInPlayList(id_moving);


        input = {"oldpos":pos_inicial,"newpos":pos_final};
        jQuery.post("xmms2/moveSong",input,function(data){
        });
    });


    jQuery("#file").change(function () {
        alert("W");
        //jQuery("#file_selector").trigger('click');
    });


}

function deleteSong(id){
    jQuery('*').css('cursor', 'progress');
    param = {"ident":id};
    jQuery.getJSON("xmms2/delete",param,function(data){
        jQuery('*').css('cursor', 'auto');
        fillList(data);
    });

}


function getPosInPlayList(id){
    for(i=0;i<play_list.length;i++){
        if(play_list[i]==id){
            return i;
            break;
        }
    }
    return -1;
}

function scheduled(){

    //If there is a song playing, we change its icon
    jQuery.post("xmms2/getSelectedSong",function(data){
        jQuery(".play_icon").hide();
        if(data!=-1){
            jQuery("#play_icon_"+data).show();
        }
    });
    toSend={"volume":jQuery("#volume").slider("value")};
    jQuery.post("xmms2/setVolume",toSend,function(data){});

}


function fillList(data){

        jQuery("#music_list").html("");

        jQuery.each(data, function(key, val) {

            jQuery("#music_list").append('<li class="ui-state-default" ondblclick="playSong('+val.id+')" id="'+val.id+'">  <table style="width:100%;"><tr>  <td style="width: 15px;" > <img id="play_icon_'+val.id+'" src="/xmms2/static/img/play_icon.png" class="play_icon"></img> </td> <td style="width: 715px;"> <p style="width:715px; margin:0; word-wrap:break-word; overflow: hidden;height: 20px;line-height: 20px;  ">'+val.title+' ('+val.album+') - '+val.artist+' </p>   </td> <td> <div onclick="deleteSong('+val.id+')" style="float:right;" class="ui-icon ui-icon-trash delete-buttons" ></div>   </td>    </tr>          </table>             </li> ');

        });

        play_list = jQuery("#music_list").sortable('toArray');

    

}

function playSong(id){
    param = {"ident":id};
    jQuery.post("xmms2/playSong",param,function(data){
//        jQuery("#playpause").trigger("click");
    });
}
