/*
* Software License Agreement (GPLv2 License)
* 
* Copyright (c) 2011 Thecorpora, S.L.
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
* Author: Daniel Cuadrado Sánchez <daniel.cuadrado@openqbo.com>
*/


package com.thecorpora.qbo.androidapk;

import android.app.Activity;
import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.content.pm.ResolveInfo;
import android.graphics.drawable.BitmapDrawable;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.preference.PreferenceManager;
import android.speech.RecognizerIntent;
import android.util.DisplayMetrics;
import android.util.Log;
import android.view.*;
import android.view.View.OnClickListener;
import android.view.View.OnTouchListener;
import android.media.AudioManager;
import android.net.ConnectivityManager;
import android.net.NetworkInfo;
import android.net.sip.*;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.ProgressBar;
import android.widget.RelativeLayout;
import android.widget.TextView;
import android.widget.Toast;
import java.text.ParseException;
import java.util.ArrayList;
import java.util.List;

/**
 * 
 * Here you will find the main functionality of the Qbo Android Application:
 * 
 *  -> Showing UI
 *  -> Getting user inputs (joysticks, buttons, ...)
 *  -> Displaying images from Qbo webcams (http://www.ros.org/wiki/mjpeg_server)
 *  -> Managing SIP functionality
 *     
 *  Feel free to change and/or improve whatever you want.
 *  
 * @author Daniel Cuadrado Sanchez
 *
 */
public class MainActivity extends Activity {


	private static final String TAG = "AndroidQbo";
	
    public String sipAddress = null;

    public SipManager manager = null;
    public SipProfile me = null;
    public SipAudioCall call = null;

    private static final int UPDATE_SETTINGS_DIALOG = 3; 
	
	private String ip,
				   oldIp="", /* if ip is changed, we need to restart cookie and get user and password again */				   
				   language,
				   output="",
				   text,
				   userSipName="",
				   botSipName="",
				   quality="";
				
	
	private long delay,
				 startTime;

	private boolean language_recognition,
				    debug,
				    stopProcess,
				    multitouch,
				    clickOnBody=false, 
				    clickOnHead=false, 
				    setFace=false, 
				    setText=false, 
				    setSquare=false, 
				    threadCmdOn,
				     
				    sipOn = false,				    
					threeDOn = false, 
					showFps = true;
	

	private int imageType,
				VOICE_RECOGNITION_REQUEST_CODE=1,
				face,
				lastface = -1,
				screenW,
				screenH,
				numFingersBefore,
				posReposeBodyX,
				posReposeBodyY,
				posReposeHeadX,
				posReposeHeadY,
				posInitX = 0,
				posInitY = 0,
				posX = 0,
				posY = 0,
				timeClick = 300;
	

	private Button acceptText2Speech,
				   cancelText2Speech;


	private ImageView joystickBody,
					  joystickHead,
					  button_voice,
					  button_3d,
					  button_emoticons, 
					  iv_smile,
					  iv_sad,
					  iv_plain,
					  iv_angry,
					  iv_surprised,
					  iv_laught,
					  iv_funny,
					  iv_cry,connection_quality_1,connection_quality_2,connection_quality_3,connection_quality_4,connection_quality_5;


	private double outputBodyX,
				   outputBodyY,
				   outputHeadX,
				   outputHeadY,
				   linearSpeed,
				   angularSpeed;
	
	private RelativeLayout rl_head_joystick,
						   layoutBody,
						   layoutHead,
						   rl_body_joystick,
						   rl_insert_text,
						   rl_square, 
						   rl_emoticons,
						   rl_buttons_vertical,
						   rl_buttons,
						   rl_about_us;
	
	
	private Thread cThreadCmd;	

	private EditText editText_text2Speech;

	private TextView debugXBody, 
					 debugYBody,
					 debugXHead, 
					 debugYHead;
		
	private ProgressBar waiting;

	private RESTClient rest=null;

	private AudioManager audioManager;
	
	private HeadPhones_Receiver headPhoneReceiver;	
	
	public Context ctx;

	private MjpegView mjpegview=null;
	private RelativeLayout mainLayout;
	
	private Menu optionsMenu=null;
	
	private ConnectivityManager managerConnectivity;


	private IntentFilter filter;
	
	private String actualUrlImg;

	  //////////////////////////////////////////////////////////
	 //       Android Flow (onCreate, onStart, etc)          //
	//////////////////////////////////////////////////////////
	
    @Override
    public void onCreate(Bundle savedInstanceState) {    	     
        super.onCreate(savedInstanceState);
                
        if (!SipManager.isVoipSupported(this)){
        	Toast.makeText(getBaseContext(),R.string.voip_not_supported,Toast.LENGTH_SHORT).show();
        }
        if (!SipManager.isApiSupported(this)){
        	Toast.makeText(getBaseContext(),R.string.sip_api_not_supported,Toast.LENGTH_SHORT).show();
        }
        
        managerConnectivity = (ConnectivityManager)getSystemService(this.CONNECTIVITY_SERVICE);
      
        setContentView(R.layout.main);  
		ctx=this;

		// Keep screen always on
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

		mainLayout = (RelativeLayout) findViewById(R.id.RelativeLayout); 

		acceptText2Speech = (Button) findViewById(R.id.acceptText);
		acceptText2Speech.setOnClickListener(cliclListener4acceptText2Speech);

		cancelText2Speech = (Button) findViewById(R.id.cancelText);
		cancelText2Speech.setOnClickListener(cliclListener4cancelText2Speech);

		iv_smile  = (ImageView)findViewById(R.id.iv_smile);
		iv_smile.setOnClickListener(onClickListener4iv_smile);

		iv_sad  = (ImageView)findViewById(R.id.iv_sad);
		iv_sad.setOnClickListener(onClickListener4iv_sad);

		iv_plain  = (ImageView)findViewById(R.id.iv_plain);
		iv_plain.setOnClickListener(onClickListener4iv_plain);

		iv_angry  = (ImageView)findViewById(R.id.iv_angry);
		iv_angry.setOnClickListener(onClickListener4iv_angry);

		iv_surprised  = (ImageView)findViewById(R.id.iv_surprised);
		iv_surprised.setOnClickListener(onClickListener4iv_surprised);

		iv_cry  = (ImageView)findViewById(R.id.iv_cry);
		iv_cry.setOnClickListener(onClickListener4iv_cry);

		iv_laught  = (ImageView)findViewById(R.id.iv_laught);
		iv_laught.setOnClickListener(onClickListener4iv_laught);

		iv_funny  = (ImageView)findViewById(R.id.iv_funny);
		iv_funny.setOnClickListener(onClickListener4iv_funny);
		
				
		connection_quality_1 = (ImageView)findViewById(R.id.connection_quality_1);
		connection_quality_2 = (ImageView)findViewById(R.id.connection_quality_2);
		connection_quality_3 = (ImageView)findViewById(R.id.connection_quality_3);
		connection_quality_4 = (ImageView)findViewById(R.id.connection_quality_4);
		connection_quality_5 = (ImageView)findViewById(R.id.connection_quality_5);
		connection_quality_1.setVisibility(View.GONE);
		connection_quality_2.setVisibility(View.GONE);
		connection_quality_3.setVisibility(View.GONE);
		connection_quality_4.setVisibility(View.GONE);		
		

		layoutBody = (RelativeLayout)findViewById(R.id.layoutBody);
		

		rl_insert_text = (RelativeLayout)findViewById(R.id.rl_insert_text);

		rl_square = (RelativeLayout)findViewById(R.id.rl_square);

		rl_emoticons = (RelativeLayout)findViewById(R.id.rl_emoticons);

		editText_text2Speech = (EditText)findViewById(R.id.editext_text);

		debugXBody = (TextView) findViewById(R.id.debugXBody);
		debugYBody = (TextView) findViewById(R.id.debugYBody);		
		debugXHead = (TextView) findViewById(R.id.debugXHead);
		debugYHead = (TextView) findViewById(R.id.debugYHead);
		
		//Screen size
		Display display = getWindowManager().getDefaultDisplay();
		screenW = display.getWidth();
		screenH = display.getHeight(); 

		joystickBody=(ImageView) findViewById(R.id.joystickBody);
		joystickHead=(ImageView) findViewById(R.id.joystickHead);

		rl_body_joystick = (RelativeLayout )findViewById(R.id.rl_body_joystick);
		rl_head_joystick = (RelativeLayout )findViewById(R.id.rl_head_joystick);

		rl_about_us = (RelativeLayout )findViewById(R.id.rl_about_us);
		rl_about_us.setOnClickListener(rl_about_usListener);

		layoutBody.setOnTouchListener(joystickBodyListener);		
			
		layoutBody.setClickable(true);



		//joystick positions
		BitmapDrawable bd=(BitmapDrawable) this.getResources().getDrawable(R.drawable.joystick_body);
		DisplayMetrics dm = new DisplayMetrics();
		getWindowManager().getDefaultDisplay().getMetrics(dm);

		//will either be DENSITY_LOW, DENSITY_MEDIUM or DENSITY_HIGH
		int dpiClassification = dm.densityDpi;
		bd.setTargetDensity(dpiClassification);
		int bodyJoystickHeight=bd.getBitmap().getHeight();
		int bodyJoystickWidth=bd.getBitmap().getWidth();
		
	
		bd=(BitmapDrawable) this.getResources().getDrawable(R.drawable.joystick_head);
		bd.setTargetDensity(dpiClassification);
		int headJoystickHeight=bd.getBitmap().getHeight();
		int headJoystickWidth=bd.getBitmap().getWidth();
		
		posReposeBodyX=(layoutBody.getLayoutParams().width/2)-(bodyJoystickWidth/2); // 30 = widthJoystick/2 
		posReposeBodyY=screenH-( (layoutBody.getLayoutParams().height/2))-(bodyJoystickHeight/2);	
		
		rl_body_joystick.setPadding(posReposeBodyX,posReposeBodyY,0,0);	
		

		button_voice = (ImageView)findViewById(R.id.button_voice);
		button_voice.setOnClickListener(clickListener4Button_voice);
		button_voice.setSelected(false);

		button_emoticons = (ImageView)findViewById(R.id.button_emoticons);
		button_emoticons.setOnClickListener(clickListener4Button_emoticons);


		//In case the device is multitouch, the head joystick will be shown at the bottom of the screen.
		PackageManager pm = getPackageManager();
		if(pm.hasSystemFeature(PackageManager.FEATURE_TOUCHSCREEN_MULTITOUCH_DISTINCT)){
			layoutHead = (RelativeLayout)findViewById(R.id.layoutHead);

			posReposeHeadX=screenW-(layoutHead.getLayoutParams().width/2)-(headJoystickWidth/2);
			posReposeHeadY=screenH-(layoutHead.getLayoutParams().height/2)-(headJoystickHeight/2);
			rl_head_joystick.setPadding(posReposeHeadX,posReposeHeadY,0,0);				

			rl_buttons_vertical = (RelativeLayout)findViewById(R.id.rl_buttons_vertical);
			rl_buttons_vertical.setVisibility(View.VISIBLE);
			rl_buttons = (RelativeLayout)findViewById(R.id.rl_buttons);
			rl_buttons.setVisibility(View.GONE);

			button_voice = (ImageView)findViewById(R.id.button_voiceV);
			button_voice.setOnClickListener(clickListener4Button_voice);

			button_emoticons = (ImageView)findViewById(R.id.button_emoticonsV);
			button_emoticons.setOnClickListener(clickListener4Button_emoticons);

			button_3d = (ImageView)findViewById(R.id.button_3dV);
			button_3d.setOnClickListener(cliclListener4button3d);

			multitouch = true;
		}else{					
			layoutHead = (RelativeLayout)findViewById(R.id.layoutHead_V);
			
			posReposeHeadX=screenW-(layoutHead.getLayoutParams().width/2)-(headJoystickWidth/2);
			posReposeHeadY=(layoutHead.getLayoutParams().height/2)-(headJoystickHeight/2);		
			rl_head_joystick.setPadding(posReposeHeadX,posReposeHeadY,0,0);

			button_voice = (ImageView)findViewById(R.id.button_voice);
			button_voice.setOnClickListener(clickListener4Button_voice);

			button_emoticons = (ImageView)findViewById(R.id.button_emoticons);
			button_emoticons.setOnClickListener(clickListener4Button_emoticons);

			button_3d = (ImageView)findViewById(R.id.button_3d);
			button_3d.setOnClickListener(cliclListener4button3d);


			multitouch = false;
		}		

		layoutHead.setVisibility(View.VISIBLE);
		layoutHead.setOnTouchListener(joystickHeadListener);	
		
		waiting = (ProgressBar) findViewById(R.id.progressBar1);
		waiting.setVisibility(View.GONE);
		
		audioManager = (AudioManager) getSystemService(Context.AUDIO_SERVICE); 
       
		//Service for capturing HEADPHONES plug-in
		filter = new IntentFilter("android.intent.action.HEADSET_PLUG");
        filter.addCategory(Intent.CATEGORY_DEFAULT);
        headPhoneReceiver = new HeadPhones_Receiver();
    }

    @Override
    public void onStart() {
        super.onStart();       
       
        // Get info from settings
		SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(getBaseContext());                
		ip = prefs.getString("ip", "192.168.4.6");	
		prefs.getString("portSip", "5060");
		delay = Long.valueOf(prefs.getString("delay", "10"));
		imageType = 0; 
		
		quality =  prefs.getString("quality", "50");

		language = String.valueOf(prefs.getString("language", "en"));
		language_recognition = prefs.getBoolean("checkbox_language_recognition", true); 
		debug = prefs.getBoolean("debug", false);
		linearSpeed = Double.valueOf(prefs.getString("linearSpeed", "0.3"));
		angularSpeed = Double.valueOf(prefs.getString("angularSpeed", "0.8"));		

		if(debug){
			debugXBody.setVisibility(View.VISIBLE);
			debugYBody.setVisibility(View.VISIBLE);
			debugXHead.setVisibility(View.VISIBLE);
			debugYHead.setVisibility(View.VISIBLE);
		}else{
			debugXBody.setVisibility(View.GONE);
			debugYBody.setVisibility(View.GONE);
			debugXHead.setVisibility(View.GONE);
			debugYHead.setVisibility(View.GONE);
		}    
        
    }
    
    @Override
    protected void onResume(){
    	super.onResume();
    	try{
    		//we get the username from the linphone in the bot
    		rest = new RESTClient(ip, this);
    		rest.setImgQuality(quality);

    		if(ip!=oldIp && !oldIp.equalsIgnoreCase("") ){
    			rest.setCookie(null);
    		}    		
    		
    		boolean is3g = managerConnectivity.getNetworkInfo(ConnectivityManager.TYPE_MOBILE).isConnectedOrConnecting();
    		boolean isWifi = managerConnectivity.getNetworkInfo(ConnectivityManager.TYPE_WIFI).isConnectedOrConnecting();
    		
    		if(!is3g && !isWifi){
    			Toast.makeText(getBaseContext(),R.string.nor_3g_neither_wifi,Toast.LENGTH_LONG).show();
    			go2Login();
    		}    		

    		if (rest.hasCookie()){ 
    				stopProcess=false;	

    				actualUrlImg = rest.getURL_IMG(imageType);    				
    				
    				mjpegview = new MjpegView(this);  
    				
    				new AsyncTask<String, Void, Void>() {
    					@Override
    					protected Void doInBackground(String... arg0) {
    						mjpegview.setSource(MjpegInputStream.read(arg0[0],rest.getCookie()));
    						return null;								
    					}	
    					protected void onPostExecute(Void arg0) {
    						mjpegview.setDisplayMode(MjpegView.SIZE_BEST_FIT);
    	    				mjpegview.showFps(showFps);
    	    				try{
    	    					mainLayout.removeView(mjpegview);
    	    				}catch(Exception e){
    	    					//do nothing. Since we are changing to asynchronous tasks we can have different floworks
    	    				}
    	    				mainLayout.addView(mjpegview, 0);    	    				
    	    				startThread(); 
    					}
    				}.execute(actualUrlImg);    				
    				    			
    		}else{    			
    			go2Login();    			
    		}

    	}catch(Exception e){		
    		Log.e(TAG,e.toString());
    		go2Login();    		
    	}
    	
    	registerReceiver(headPhoneReceiver, filter);
    }
    
	@Override
	protected void onPause(){
		super.onPause();			
		
		//Stop headphones service, getting images, threads  		
		if(sipOn){
			CloseSIPServer closeSip = new CloseSIPServer();
			closeSip.execute();		
		}	

		if (mjpegview!=null){
			mjpegview.stopPlayback();
			mainLayout.removeView(mjpegview);
		}		
		
		//We tell to the server to stop streaming video
		new AsyncTask<Void, Void, Void>() {
			@Override
			protected Void doInBackground(Void... arg0) {
				rest.stopVideo(actualUrlImg);
				return null;								
			}		
		}.execute();
        
        StopComunication stopComunication = new StopComunication();
        stopComunication.execute(false);
                		
		//we remember the actual Ip
		oldIp=ip;		

		stopThread();
		unregisterReceiver(headPhoneReceiver);
	}
    
    @Override
    public void onDestroy() {
    	try{
    		unregisterReceiver(headPhoneReceiver);
    	}catch(IllegalArgumentException iae){
    		Log.e(TAG,iae.toString());
    	}
    	
    	super.onDestroy(); 

    	if (call != null) {
    		call.close();
    	}

    	try{
    		audioManager.setSpeakerphoneOn(false);
    	}catch(Exception e){
    		Log.e(TAG,e.toString());
    	}

    }

    
    
    
	  //////////////////////////////////////////////////////////
	 //                  Other Functions                     //
	//////////////////////////////////////////////////////////
	
	private void startThread(){   
		cThreadCmd = new Thread(new ThreadCmd());
		stopProcess=false;
		cThreadCmd.start();	
		threadCmdOn = true;

	}

	private void stopThread(){
		if(threadCmdOn){			
			stopProcess=true;
			try {
				cThreadCmd.join(1);
				threadCmdOn=false;
			} catch (InterruptedException e) {           
				e.printStackTrace();
			} 
		}		
	}

    //This just launches the activity responsible of getting the ip robot and making the security user check
    private void go2Login(){
    	Intent intent = new Intent(this, LogInActivity.class);			 
		this.startActivity(intent);
    }

	  //////////////////////////////////////////////////////////
	 //                  Events                              //
	//////////////////////////////////////////////////////////

	// When we want Qbo reproduce whatever we say, there are two options: using Google Recognition or writing in an box.
	private OnClickListener clickListener4Button_voice = new OnClickListener(){

		@Override
		public void onClick(View v) {				

			//Checking if we have internet 
			ConnectivityManager connectivityManager = (ConnectivityManager) getSystemService(Context.CONNECTIVITY_SERVICE);
			NetworkInfo activeNetworkInfo = connectivityManager.getActiveNetworkInfo();

			// Checking whether recognition activity is available in this device or not
			PackageManager pm = getPackageManager();
			List<ResolveInfo> activities = pm.queryIntentActivities(
					new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH), 0);

			if (activities.size() != 0  && activeNetworkInfo != null && language_recognition) {
				startVoiceRecognition();
			} else {	        	
				showInsertTextLayout();	        	
			}				
		}

	};


	private void showInsertTextLayout(){
		if(rl_insert_text.getVisibility() == View.VISIBLE){
			rl_insert_text.setVisibility(View.GONE);
		}else {
			if(language_recognition) Toast.makeText(getBaseContext(),R.string.speech_recognition_not_available,Toast.LENGTH_LONG).show();

			editText_text2Speech.setText("");

			rl_insert_text.setVisibility(View.VISIBLE);		
			rl_emoticons.setVisibility(View.GONE);
//			rl_square_detection.setVisibility(View.GONE);	
//			rl_square.setVisibility(View.GONE);
		}
	}

	private OnClickListener cliclListener4acceptText2Speech = new OnClickListener(){
		@Override
		public void onClick(View v) {						
			text = editText_text2Speech.getText().toString();			
			setText=true;
			rl_insert_text.setVisibility(View.GONE);			
		}		
	};

	private OnClickListener cliclListener4cancelText2Speech = new OnClickListener(){
		@Override
		public void onClick(View v) {
			rl_insert_text.setVisibility(View.GONE);			
		}		
	};

	private OnClickListener cliclListener4button3d = new OnClickListener(){
		@Override
		public void onClick(View v) {
			waiting.setVisibility(View.VISIBLE);
			
			mjpegview.stopPlayback();
			mainLayout.removeView(mjpegview);

			new AsyncTask<String, Void, Void>() {
				@Override
				protected Void doInBackground(String... url) {					
					rest.stopVideo(url[0]);
					return null;								
				}	
				
				@Override
				protected void onPostExecute(Void arg0) {
				}
				
			}.execute(actualUrlImg);
			
			
			if (!threeDOn){				
				new AsyncTask<Void,Void,Void>(){
					@Override
					protected Void doInBackground(Void... params) {
						rest.post_activate_3D("on");
						return null;
					}
					@Override
					protected void onPostExecute(Void arg0) {
					}
				}.execute();				
				imageType=7;
				threeDOn = true;
				actualUrlImg = rest.getURL_IMG(imageType);
				
			}else{				
				threeDOn = false;
				//Shut down ROS node.
				new AsyncTask<Void,Void,Void>(){
					@Override
					protected Void doInBackground(Void... params) {
						rest.post_activate_3D("off");
						return null;
					}	
					@Override
					protected void onPostExecute(Void arg0) {
						int a=3;
						a=a+2;
					}
				}.execute();					
				imageType=0;
				actualUrlImg = rest.getURL_IMG(imageType);				
			}
			
			mjpegview = new MjpegView(ctx);
			
			new AsyncTask<String, Void, Void>() {
				@Override
				protected Void doInBackground(String... arg0) {
					mjpegview.setSource(MjpegInputStream.read(arg0[0],rest.getCookie()));
					return null;								
				}	
				protected void onPostExecute(Void arg0) {
					mjpegview.setDisplayMode(MjpegView.SIZE_BEST_FIT);
    				mjpegview.showFps(showFps);
    				mainLayout.addView(mjpegview, 0); 
    				waiting.setVisibility(View.GONE);
				}
			}.execute(actualUrlImg); 
		}						
	};	


	private OnClickListener clickListener4Button_emoticons = new OnClickListener(){
		@Override
		public void onClick(View v) {	
			if(rl_emoticons.getVisibility() == View.VISIBLE){
				rl_emoticons.setVisibility(View.GONE);
			}else{
				rl_emoticons.setVisibility(View.VISIBLE);	
//				rl_square_detection.setVisibility(View.GONE);		
				rl_insert_text.setVisibility(View.GONE);
				rl_square.setVisibility(View.GONE);
			}
		}						
	};		

	private OnClickListener rl_about_usListener = new OnClickListener(){
		@Override
		public void onClick(View v) {	
			rl_about_us.setVisibility(View.GONE);
		}						
	};		

	// ********** Click on faces  ****************  //
	private OnClickListener onClickListener4iv_smile = new OnClickListener(){
		@Override
		public void onClick(View v) {		
			setFace=true;

			if(lastface == 0){
				face = 13;
			}else{
				face=0;
			}			

			rl_emoticons.setVisibility(View.GONE);
		}						
	};		

	private OnClickListener onClickListener4iv_sad = new OnClickListener(){
		@Override
		public void onClick(View v) {		
			setFace=true;
			if(lastface == 1){
				face = 13;
			}else{
				face=1;
			}		
			rl_emoticons.setVisibility(View.GONE);
		}						
	};

	private OnClickListener onClickListener4iv_plain = new OnClickListener(){
		@Override
		public void onClick(View v) {		
			setFace=true;
			if(lastface == 5){
				face = 13;
			}else{
				face=5;
			}		
			rl_emoticons.setVisibility(View.GONE);
		}						
	};

	private OnClickListener onClickListener4iv_angry = new OnClickListener(){
		@Override
		public void onClick(View v) {		
			setFace=true;
			if(lastface == 4){
				face = 13;
			}else{
				face=4;
			}		
			rl_emoticons.setVisibility(View.GONE);
		}						
	};

	private OnClickListener onClickListener4iv_surprised = new OnClickListener(){
		@Override
		public void onClick(View v) {		
			setFace=true;
			if(lastface == 2){
				face = 13;
			}else{
				face=2;
			}
			rl_emoticons.setVisibility(View.GONE);
		}						
	};

	private OnClickListener onClickListener4iv_cry = new OnClickListener(){
		@Override
		public void onClick(View v) {		
			setFace=true;
			if(lastface == 6){
				face = 13;
			}else{
				face=6;
			}		
			rl_emoticons.setVisibility(View.GONE);
		}						
	};

	private OnClickListener onClickListener4iv_laught = new OnClickListener(){
		@Override
		public void onClick(View v) {		
			setFace=true;
			if(lastface == 10){
				face = 13;
			}else{
				face=10;
			}			
			rl_emoticons.setVisibility(View.GONE);
		}						
	};

	private OnClickListener onClickListener4iv_funny = new OnClickListener(){
		@Override
		public void onClick(View v) {		
			setFace=true;
			if(lastface == 16){
				face = 13;
			}else{
				face=16;
			}		
			rl_emoticons.setVisibility(View.GONE);
		}						
	};
	// ********** end faces  ****************  //







	  //////////////////////////////////////////////////////////
	 //                  Options Menu                        //
	//////////////////////////////////////////////////////////
	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		super.onCreateOptionsMenu(menu);		

		MenuItem menuOptions = menu.add(0,0,0,R.string.disconnect);
		menuOptions.setIcon(R.drawable.disconnected);	

		menuOptions = menu.add(2,1,0,R.string.telepresenceOn);
		menuOptions.setIcon(R.drawable.telepresence_on);	

		menuOptions = menu.add(1,1,0,R.string.telepresenceOff);
		menuOptions.setIcon(R.drawable.telepresence_off);	

		menuOptions= menu.add(0,2,1,R.string.settings);
		menuOptions.setIcon(R.drawable.settings);

		menuOptions= menu.add(0,3,1,R.string.about);  
		menuOptions.setIcon(R.drawable.about);
		return true;
	}

	@Override
	public boolean onPrepareOptionsMenu(Menu menu) {
		if (optionsMenu==null) optionsMenu=menu;
		
		if(sipOn){
			menu.setGroupVisible(1,true);
			menu.setGroupVisible(2,false);
		}else{
			menu.setGroupVisible(1,false);
			menu.setGroupVisible(2,true);	
		}
		
		return super.onPrepareOptionsMenu(menu);
	}

	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		super.onOptionsItemSelected(item);

		switch (item.getItemId()) {
		case 0:
	        StopComunication stopComunication_logout = new StopComunication();
	        stopComunication_logout.execute(true);	        
			stopThread();
			
			Toast.makeText(getBaseContext(),R.string.disconnect,Toast.LENGTH_SHORT).show();
			go2Login();			
			break;
		case 1:	
			waiting.setVisibility(View.VISIBLE);
			if(sipOn){			
				//Close SIP
				CloseSIPServer closeSip = new CloseSIPServer();
				closeSip.execute();
				
			}else{			
				//Start SIP
			    StartSIPServer startSip = new StartSIPServer();
			    startSip.execute();				
			}	

			break;
		case 2:	
			Intent i = new Intent(this, QboPreferences.class);
			startActivity(i); 

			break;
		case 3: 
			rl_about_us.setVisibility(View.VISIBLE);
			break;  
		}


		return false;
	}
	// ********** End Options Menu  ****************  //



	// When back button is pressed, we go back to home screen, or we turn off the "about us" screen in case it was on
	@Override
	public void onBackPressed() {
		if(!rl_about_us.isShown()){
			//exit app, going home
			Intent intent = new Intent(Intent.ACTION_MAIN);
			intent.addCategory(Intent.CATEGORY_HOME);
			intent.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
			startActivity(intent);
		}else{
			rl_about_us.setVisibility(View.GONE);
		}
		return;
	}



	  //////////////////////////////////////////////////////////
	 //                  Joysticks                           //
	//////////////////////////////////////////////////////////

	// Both body and head joystick are images that can be moved, we will get their local position to get the input moves. If they are out of certain bounds (the joystick base circle), 
	//they will stop their move, or follow the contour of the circle.
	//
	// We have to keep in mind that several combinations might happen:
	// - Just head is moving, no problem
	// - Just body is moving, no problem
	// - While head is moving, body is then moving as well, then it is the head listener the one who will take care of this
	// - Viceversa
	// - If while moving head, body is then touched (the case we just mentioned), and then head is out, it will be still the head listener the one who will take care
	// - Viceversa
	//
	// For double tap we count the miliseconds between ACTION_DOWN and ACTION_UP	
	private OnTouchListener joystickBodyListener = new OnTouchListener(){

		@Override
		public boolean onTouch(View v, MotionEvent event) {	


			if( event.getAction() == MotionEvent.ACTION_DOWN){
				startTime = System.currentTimeMillis();			
			}

			if( event.getAction() == MotionEvent.ACTION_UP){			
				rl_head_joystick.setPadding(posReposeHeadX,posReposeHeadY,0,0);

				outputBodyX=0;
				outputBodyY=0;	
				rl_body_joystick.setPadding(posReposeBodyX,posReposeBodyY,0,0);

				numFingersBefore=0;

				return true;				
			}else if(event.getAction() == MotionEvent.ACTION_MOVE){				
				int radio=40;
				if(event.getPointerCount()==1){		
					if(numFingersBefore==2){
						// There was TWO finger touching the screen, one is up now...which one?
						numFingersBefore=1;
						if(event.getX() > screenW/2){
							// The finger which was on BODY
							outputBodyX=0;
							outputBodyY=0;
							if( System.currentTimeMillis()-startTime < timeClick){ //just on Click
								System.out.println("DEDO UPPPPP");	
								clickOnBody = true;
							}
							rl_body_joystick.setPadding(posReposeBodyX,posReposeBodyY,0,0);

						}else{
							// The finger which was on HEAD
							if( System.currentTimeMillis()-startTime < timeClick){  //just on Click
								clickOnHead = true;
							}
							rl_head_joystick.setPadding(posReposeHeadX,posReposeHeadY,0,0);

						}						
					}
					numFingersBefore=1;

					if(event.getX() > screenW/2){ 
						//In this case we have just ONE finger moving the head joystick						
						double origenX2 = layoutHead.getWidth()/2;
						double origenY2 = layoutHead.getHeight()/2;

						double x22=(int)event.getX();
						double y22=(int)event.getY();

						if(!multitouch) y22 = y22 + screenH - layoutBody.getHeight(); //Move from body origin to the main origin coordinate system						
						x22 = x22 - (screenW - layoutHead.getWidth()); //From main origin to head origin.

						double x2=x22-origenX2;
						double y2=y22-origenY2;

						double[] auxD2 = circulePoints(x2,y2,radio);
						x2=auxD2[0];
						y2=auxD2[1];

						outputHeadX=(int)x2;
						outputHeadY=(int)y2;

						x2=x2+origenX2;
						y2=y2+origenY2;  

						x2=x2+ screenW - layoutHead.getWidth();
						if(multitouch) y2=y2+ screenH - layoutHead.getHeight();

						rl_head_joystick.setPadding((int)x2-(joystickHead.getWidth()/2), (int)y2-(joystickHead.getHeight()/2), 0, 0);
						outputBodyX=0;
						outputBodyY=0;
						rl_body_joystick.setPadding(posReposeBodyX, posReposeBodyY , 0, 0);

					}else{
						//ONE finger moving body joystick							
						double origenX = layoutBody.getWidth()/2;
						double origenY = layoutBody.getHeight()/2;

						double x1=(int)event.getX()-origenX;
						double y1=(int)event.getY()-origenY;

						double[] auxD = circulePoints(x1,y1,radio);
						x1=auxD[0];
						y1=auxD[1];

						outputBodyX=(int)x1;
						outputBodyY=(int)y1;


						x1=x1+origenX;
						y1=y1+origenY;            

						y1=y1+ screenH - layoutBody.getHeight();


						//joystickBody.layout((int)x1-(joystickBody.getWidth()/2), (int)y1-(joystickBody.getHeight()/2), (int)x1+(joystickBody.getWidth()/2), (int)y1+(joystickBody.getHeight()/2));						
						rl_body_joystick.setPadding((int)x1-(joystickBody.getWidth()/2), (int)y1-(joystickBody.getHeight()/2), 0, 0);

						rl_head_joystick.setPadding(posReposeHeadX, posReposeHeadY , 0, 0);
					}
	
				}else if(event.getPointerCount() == 2){
					if(numFingersBefore == 1){
						startTime = System.currentTimeMillis(); //It might happen that, while moving body, one click is made on head
					}
					numFingersBefore=2;
					double x11=(int)event.getX(0);
					double y11=(int)event.getY(0);
					double x22=(int)event.getX(1);
					double y22=(int)event.getY(1);

					double origenX1 = layoutBody.getWidth()/2;
					double origenY1 = layoutBody.getHeight()/2;
					double origenX2 = layoutHead.getWidth()/2;
					double origenY2 = layoutHead.getHeight()/2;


					if(!multitouch) y22 = y22 + screenH - layoutBody.getHeight(); //From body origin to main origin

					x22 = x22 - (screenW - layoutHead.getWidth()); //from main origin to head origin


					double x1=x11-origenX1;
					double y1=y11-origenY1;

					double x2=x22-origenX2;
					double y2=y22-origenY2;


					double[] auxD = circulePoints(x1,y1,radio);
					x1=auxD[0];
					y1=auxD[1];

					outputBodyX=(int)x1;
					outputBodyY=(int)y1;

					double[] auxD2 = circulePoints(x2,y2,radio);
					x2=auxD2[0];
					y2=auxD2[1];

					outputHeadX=(int)x2;
					outputHeadY=(int)y2;

					x1=x1+origenX1;
					y1=y1+origenY1;       
					x2=x2+origenX2;
					y2=y2+origenY2; 

					y1=y1+ screenH - layoutBody.getHeight();
					x2=x2+ screenW - layoutHead.getWidth();
					if(multitouch) y2=y2+ screenH - layoutHead.getHeight();

					rl_body_joystick.setPadding((int)x1-(joystickBody.getWidth()/2), (int)y1-(joystickBody.getHeight()/2), 0, 0);

					rl_head_joystick.setPadding((int)x2-(joystickHead.getWidth()/2), (int)y2-(joystickHead.getHeight()/2), 0, 0);

				}
			}
			return true;
		}
	};  



	private OnTouchListener joystickHeadListener = new OnTouchListener(){

		@Override
		public boolean onTouch(View v, MotionEvent event) {

			if( event.getAction() == MotionEvent.ACTION_DOWN){
				startTime = System.currentTimeMillis();				 
			}

			if(event.getAction() == MotionEvent.ACTION_UP){

				System.out.println("DEDO head UP");

				rl_head_joystick.setPadding(posReposeHeadX,posReposeHeadY,0,0);

				outputBodyX=0;
				outputBodyY=0;	
				rl_body_joystick.setPadding(posReposeBodyX,posReposeBodyY,0,0);

				numFingersBefore=0;		

				if( System.currentTimeMillis()-startTime < timeClick){ //We have make one click
					clickOnHead = true;
				}
				return true;				
			}else if(event.getAction() == MotionEvent.ACTION_MOVE){			
				int radio=40;
				if(event.getPointerCount()==1){
					if(numFingersBefore==2){
						numFingersBefore=1;
						if(event.getX() < (screenW/2)*-1){//Finger up from head
							if( System.currentTimeMillis()-startTime < timeClick){ //and it was one click
								clickOnHead = true;
							}
							rl_head_joystick.setPadding(posReposeHeadX,posReposeHeadY,0,0);

						}else{//Finger up from body
							outputBodyX=0;
							outputBodyY=0;
							if( System.currentTimeMillis()-startTime < timeClick){ //and it was one click								
								clickOnBody = true;
							}
							rl_body_joystick.setPadding(posReposeBodyX,posReposeBodyY,0,0);

						}						
					}else{
						if( System.currentTimeMillis()-startTime < timeClick){ //it was one click					
							clickOnBody = true;
						}
					}
					numFingersBefore=1;


					if(event.getX() < (screenW/2)*-1){ //ONE finger on the screen, and it is over the head joystick
						double origenX = layoutBody.getWidth()/2;
						double origenY = layoutBody.getHeight()/2;

						double x11=(int)event.getX();
						double y11=(int)event.getY();

						x11 = x11 + (screenW - layoutHead.getWidth()); //From head origin to main origin						
						if(!multitouch) y11 = y11 - (screenH - layoutBody.getHeight()); //From main origin to body origin	

						double x1=x11-origenX;
						double y1=y11-origenY;

						double[] auxD = circulePoints(x1,y1,radio);
						x1=auxD[0];
						y1=auxD[1];
						outputBodyX=(int)x1;
						outputBodyY=(int)y1;

						
						x1=x1+origenX;
						y1=y1+origenY;            
						
						y1=y1+ screenH - layoutBody.getHeight();


						//joystickBody.layout((int)x1-(joystickBody.getWidth()/2), (int)y1-(joystickBody.getHeight()/2), (int)x1+(joystickBody.getWidth()/2), (int)y1+(joystickBody.getHeight()/2));

						rl_body_joystick.setPadding((int)x1-(joystickBody.getWidth()/2), (int)y1-(joystickBody.getHeight()/2), 0, 0);
						
						rl_head_joystick.setPadding(posReposeHeadX, posReposeHeadY , 0, 0);

					}else{
						double origenX = layoutHead.getWidth()/2;
						double origenY = layoutHead.getHeight()/2;

						double x2=(int)event.getX()-origenX;
						double y2=(int)event.getY()-origenY;

						double[] auxD = circulePoints(x2,y2,radio);
						x2=auxD[0];
						y2=auxD[1];
						outputHeadX=(int)x2;
						outputHeadY=(int)y2;
				
						x2=x2+origenX;
						y2=y2+origenY;
					
						x2=x2+ screenW - layoutHead.getWidth();
						if(multitouch) y2=y2+ screenH - layoutHead.getHeight();

						//joystickBody.layout((int)x1-(joystickBody.getWidth()/2), (int)y1-(joystickBody.getHeight()/2), (int)x1+(joystickBody.getWidth()/2), (int)y1+(joystickBody.getHeight()/2));

						rl_head_joystick.setPadding((int)x2-(joystickHead.getWidth()/2), (int)y2-(joystickHead.getHeight()/2), 0, 0);
						outputBodyX=0;
						outputBodyY=0;
						rl_body_joystick.setPadding(posReposeBodyX, posReposeBodyY , 0, 0);
					}

				}else if(event.getPointerCount() == 2){
					if(numFingersBefore == 1){
						startTime = System.currentTimeMillis(); 
					}
					numFingersBefore=2;
					double origenX2 = layoutHead.getWidth()/2;
					double origenY2 = layoutHead.getHeight()/2;
					double origenX1 = layoutBody.getWidth()/2;
					double origenY1 = layoutBody.getHeight()/2;

					double x22=(int)event.getX(0);
					double y22=(int)event.getY(0);									
					double x11=(int)event.getX(1);
					double y11=(int)event.getY(1);


					x11 = x11 + (screenW - layoutHead.getWidth()); //From head origin to main origin

					if(!multitouch) y11 = y11 - (screenH - layoutBody.getHeight()); //from main origin to body origin

					double x2=x22-origenX2;
					double y2=y22-origenY2;
					double x1=x11-origenX1;
					double y1=y11-origenY1;

					double[] auxD2 = circulePoints(x2,y2,radio);
					x2=auxD2[0];
					y2=auxD2[1];
					outputHeadX=(int)x2;
					outputHeadY=(int)y2;

					double[] auxD = circulePoints(x1,y1,radio);
					x1=auxD[0];
					y1=auxD[1];
					outputBodyX=(int)x1;
					outputBodyY=(int)y1;

									      
					x2=x2+origenX2;
					y2=y2+origenY2;
					x1=x1+origenX1;
					y1=y1+origenY1;

					
					x2=x2+ screenW - layoutHead.getWidth();
					y1=y1+ screenH - layoutBody.getHeight();
					if(multitouch) y2=y2+ screenH - layoutHead.getHeight();


					rl_head_joystick.setPadding((int)x2-(joystickHead.getWidth()/2), (int)y2-(joystickHead.getHeight()/2), 0, 0);					
					rl_body_joystick.setPadding((int)x1-(joystickBody.getWidth()/2), (int)y1-(joystickBody.getHeight()/2), 0, 0);
	
				}
			}

			return true;
		}

	};  


	private double[] circulePoints(double x1, double y1, double radio){	
	
		double modulo = Math.sqrt(x1*x1 + y1*y1);

		if(modulo > radio){
			double m = y1/x1; 

			if (y1==0){
				double newX = radio;
				double newY=0;

				if( Math.sqrt((x1-newX)*(x1-newX) + (y1-newY)*(y1-newY) ) > Math.sqrt((x1+newX)*(x1+newX) + (y1+newY)*(y1+newY) )  ) x1 = -radio;
				else x1 =  radio;
			}if (x1==0){
				double newY = radio;
				double newX=0;

				if( Math.sqrt((x1-newX)*(x1-newX) + (y1-newY)*(y1-newY) ) > Math.sqrt((x1+newX)*(x1+newX) + (y1+newY)*(y1+newY) )  ) y1 = -radio;
				else y1 =  radio;
			}else{	            	
				double newX = Math.sqrt((radio*radio) / (m*m +1));
				double newY = m * newX;

				if( Math.sqrt((x1-newX)*(x1-newX) + (y1-newY)*(y1-newY) ) > Math.sqrt((x1+newX)*(x1+newX) + (y1+newY)*(y1+newY) )  ) x1= newX*-1;
				else x1 =  (newX);

				y1 =  (m * x1);
			}
		}

		return new double[]{x1,y1};
	}




 	  //////////////////////////////////////////////////////////
     //                  VOICE RECOGNITION                   //
	//////////////////////////////////////////////////////////    

	/*
	 * Fire an intent to start the speech recognition activity.
	 */
	private void startVoiceRecognition() {
		Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
		/*intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL,
                RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);*/

		intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE, language);
		intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_PREFERENCE, language);        
		intent.putExtra(RecognizerIntent.EXTRA_ONLY_RETURN_LANGUAGE_PREFERENCE, language);

		intent.putExtra(RecognizerIntent.EXTRA_PROMPT, "What you want me to say?");
		startActivityForResult(intent, VOICE_RECOGNITION_REQUEST_CODE);
	}

	/*
	 * Handle the results from the recognition activity.
	 */
	@Override
	protected void onActivityResult(int requestCode, int resultCode, Intent data) {
		if (requestCode == VOICE_RECOGNITION_REQUEST_CODE && resultCode == RESULT_OK) {           

			ArrayList<String> matches = data.getStringArrayListExtra(RecognizerIntent.EXTRA_RESULTS);

			Toast.makeText(getBaseContext(),matches.get(0),Toast.LENGTH_SHORT).show();
			setText=true;
			text= matches.get(0);
		}

		super.onActivityResult(requestCode, resultCode, data);
	}




	  //////////////////////////////////////////////////////////
     //               Thread for commands input              //
	//////////////////////////////////////////////////////////
	public class ThreadCmd implements Runnable { 
		
		double finalHeadX=0, finalHeadY=0;
		public void run() {
			try {                

				handler.sendEmptyMessage(0);
				
				startTime = System.currentTimeMillis();	
				while(!stopProcess){
					try{
						handler.sendEmptyMessage(1); 
						
						if(clickOnBody){
							output += "BODYRESET";								
							//rest.post_moveHead(0,0);
							clickOnBody=false;
							
						}else if(clickOnHead){
							output += "HEADRESET";							
							rest.post_moveHead(0,15);//rest.post_moveBody(0, outputHeadX);
							outputHeadX=0;
							outputHeadY=15;	
							clickOnHead=false;
							
						}else if(setFace){ // send face							
							rest.post_mouth(face);
							lastface = face;
							setFace=false;
							
						}else if(setText){// send text to say							
							rest.post_say(text);
							setText=false;
							
						}else if(setSquare){// send ROI coordinates
							output += "ROI "+posInitX+","+posInitY+","+posX+","+posY;
							setSquare=false;
							
						}else{
							//Send head movements, if face detection is not running
							double finalHeadX = outputHeadX*-12;								
							double finalHeadY = outputHeadY*12;				
							
							if (imageType <3 || imageType > 6  ) {	
								rest.post_moveHead(finalHeadX , finalHeadY);							
							}
							
							//outputBodyX and outputBodyY values are between [-40 , 40], we change to [-1 , 1] and add the factor inserted by the user
							double finalBodyX = (outputBodyX / 40) * -1 * angularSpeed;
							double finalBodyY = (outputBodyY / 40) * -1 * linearSpeed;

							//Send body movements
							rest.post_moveBody(finalBodyY,finalBodyX);	
						}

						Thread.sleep(delay);

					}catch(Exception e){
						output="ERROR al crear drawable "+e;                       
						//handler.sendEmptyMessage(-1);
					}
					
					//We set the connection quality depending on the fps given by the MjpegView
					if( System.currentTimeMillis()-startTime > 1000){ //We have make one click
						int fps = mjpegview.getFps();
						
						if (fps > 28){
							handler.sendEmptyMessage(55);
						}else if(fps > 20 ){
							handler.sendEmptyMessage(54);
						}else if(fps > 15 ){
							handler.sendEmptyMessage(53);
						}else if(fps > 10 ){
							handler.sendEmptyMessage(52);
						}else if(fps > 5 ){
							handler.sendEmptyMessage(51);
						}else{
							handler.sendEmptyMessage(50);
						}					
						
						startTime = System.currentTimeMillis();
					}
				}

		

			} catch (Exception e) {
				Log.e(TAG, "Error "+ e);
				output="Error "+e;
				handler.sendEmptyMessage(-1);				
			}


		} 

		private Handler handler = new Handler() {  
			@Override  
			public void handleMessage(Message msg) {                           
		  

				if(msg.what == 1){

				}else if (msg.what == -1){
					Log.e(TAG, "error while connecting "+output);
					Toast.makeText(getBaseContext(),"Error while connecting",Toast.LENGTH_SHORT).show();
					stopProcess=true;
				}else if (msg.what == 55){
					connection_quality_1.setVisibility(View.GONE);
					connection_quality_2.setVisibility(View.GONE);
					connection_quality_3.setVisibility(View.GONE);
					connection_quality_4.setVisibility(View.GONE);
					connection_quality_5.setVisibility(View.VISIBLE);
				}else if(msg.what == 54 ){
					connection_quality_1.setVisibility(View.GONE);
					connection_quality_2.setVisibility(View.GONE);
					connection_quality_3.setVisibility(View.GONE);
					connection_quality_4.setVisibility(View.VISIBLE);
					connection_quality_5.setVisibility(View.GONE);
				}else if(msg.what == 53 ){
					connection_quality_1.setVisibility(View.GONE);
					connection_quality_2.setVisibility(View.GONE);
					connection_quality_3.setVisibility(View.VISIBLE);
					connection_quality_4.setVisibility(View.GONE);
					connection_quality_5.setVisibility(View.GONE);
				}else if(msg.what == 52 ){
					connection_quality_1.setVisibility(View.GONE);
					connection_quality_2.setVisibility(View.VISIBLE);
					connection_quality_3.setVisibility(View.GONE);
					connection_quality_4.setVisibility(View.GONE);
					connection_quality_5.setVisibility(View.GONE);
				}else if(msg.what == 51 ){
					connection_quality_1.setVisibility(View.VISIBLE);
					connection_quality_2.setVisibility(View.GONE);
					connection_quality_3.setVisibility(View.GONE);
					connection_quality_4.setVisibility(View.GONE);
					connection_quality_5.setVisibility(View.GONE);
				}else if (msg.what == 50 ){
					connection_quality_1.setVisibility(View.GONE);
					connection_quality_2.setVisibility(View.GONE);
					connection_quality_3.setVisibility(View.GONE);
					connection_quality_4.setVisibility(View.GONE);
					connection_quality_5.setVisibility(View.GONE);
				}				
			}  
		};
	}

    
      //////////////////////////////////////////////////////
	 ////////////// Run SIP Server on Qbo /////////////////
    //////////////////////////////////////////////////////
    private class StartSIPServer extends AsyncTask<Void, Void, Integer> {
    	@Override
    	protected Integer doInBackground(Void... params) {	
    		
    		optionsMenu.getItem(1).setEnabled(false);

    		int result = rest.startSIPServer();
    		if (result>=0){
    			rest.post_ip4sip(ip);
    			userSipName = rest.getSipId().replace("\"", "").replace("\n","");		
    			botSipName= rest.getBotSipId().replace("\"", "").replace("\n","");
    			initializeManager();
    		}
    		return result;

    	}


	    @Override
	    protected void onPostExecute(Integer result) {
	    	if ( result < 0){
				//if we are right here, is probably due to a httpHostConnectException, and the method startSIPServer returned -1, we need to take the correct IP
	    		Toast.makeText(getBaseContext(),R.string.not_possible_2_connect_sip,Toast.LENGTH_SHORT).show();
			}else{		
				// When we get back from the preference setting Activity, assume
				// settings have changed, and re-login with new auth info.				
				initiateCall();
			}
			optionsMenu.setGroupVisible(1,true);			
			optionsMenu.setGroupVisible(2,false);
			    		
    		optionsMenu.getItem(1).setEnabled(true);
	    	sipOn=true;	    	
			
	    	waiting.setVisibility(View.GONE);
	    }
	}
    
    private class CloseSIPServer extends AsyncTask<Void, Void, Void> {
    	@Override
		protected Void doInBackground(Void... params) {
			rest.endCall();
			return null;
		}
    	@Override
		protected void onPostExecute(Void arg0) {
			try {					
				call.setSpeakerMode(false);
				call.endCall();						
				StopComunication stopComunication = new StopComunication();
		        stopComunication.execute(false);	  
			} catch (SipException e) {
				Log.e(TAG," ERROR when ending a call "+e);
			} catch (NullPointerException nullPointer){
				Log.e(TAG,nullPointer.toString());
			}
			
		}
	}
    	
	
	// As an input param we need a boolean saying whether we want the cookie to be removed or not	 
	private class StopComunication extends AsyncTask<Boolean, Void, Boolean> {
		@Override
		protected Boolean doInBackground(Boolean... params) {		
//			if(!params[0] && optionsMenu!=null){				
//				optionsMenu.getItem(2).setEnabled(false);
//			}
			
			closeLocalProfile();
			rest.stopSIPServer();
			
			if(params[0]){
				rest.removeCookie();
			}
			return params[0];	
		}		

	    @Override
	    protected void onPostExecute(Boolean result) {	
	    	if(!result && optionsMenu!=null ){
				optionsMenu.setGroupVisible(1,false);			
				optionsMenu.setGroupVisible(2,true);				
	    		optionsMenu.getItem(2).setEnabled(true);    		
		    	sipOn=false;
		    					
		    	waiting.setVisibility(View.GONE);
	    	}
	    }
	}	
		
    
      //////////////////////////////////////////////////////
	 //             SIP                                  //
	//////////////////////////////////////////////////////
    public void initializeManager() {
        if(manager == null) {
          manager = SipManager.newInstance(this);
        }

        initializeLocalProfile();
    }

    /**
     * Logs you into your SIP provider, registering this device as the location to
     * send SIP calls to for your SIP address.
     */
    public void initializeLocalProfile() {
        if (manager == null) {
            return;
        }

        if (me != null) {
            closeLocalProfile();
        }

        PreferenceManager.getDefaultSharedPreferences(getBaseContext());
        String username = userSipName;     
        String domain = ip;

        if (username.length() == 0 || domain.length() == 0 ) {
            showDialog(UPDATE_SETTINGS_DIALOG);
            return;
        }

        try {
            SipProfile.Builder builder = new SipProfile.Builder(username, domain); 
            
            me = builder.build();

            Intent i = new Intent();
            i.setAction("android.SipDemoTest.INCOMING_CALL");
            PendingIntent pi = PendingIntent.getBroadcast(this, 0, i, Intent.FILL_IN_DATA);
            manager.open(me,pi, null);

            // This listener must be added AFTER manager.open is called,
            // Otherwise the methods aren't guaranteed to fire.

            manager.setRegistrationListener(me.getUriString(), new SipRegistrationListener() {
                    public void onRegistering(String localProfileUri) {
                       
                    }
                    public void onRegistrationDone(String localProfileUri, long expiryTime) {

                    }
                    public void onRegistrationFailed(String localProfileUri, int errorCode,
                            String errorMessage) {
                        Log.e(TAG," SIP Error Code "+errorCode);
                    }
                });
        } catch (ParseException pe) {

        } catch (SipException se) {

        }
    }

    /**
     * Closes out your local profile, freeing associated objects into memory
     * and unregistering your device from the server.
     */
    public void closeLocalProfile() {
        if (manager == null) {
            return;
        }
        try {
            if (me != null) {
                manager.close(me.getUriString());
            }
        } catch (Exception ee) {
            Log.e(TAG, "Failed to close local profile. "+ ee);
        }
    }

    /**
     * Make an outgoing call.
     */
    public void initiateCall() {  
    	
        try {
            SipAudioCall.Listener listener = new SipAudioCall.Listener() {
                // Much of the client's interaction with the SIP Stack will
                // happen via listeners.  Even making an outgoing call, don't
                // forget to set up a listener to set things up once the call is established.
                @Override
                public void onCallEstablished(SipAudioCall call) {
                	                	 
             		 boolean headPhonesOff = audioManager.isWiredHeadsetOn();
             		 
             		 if(headPhonesOff){             			
             			audioManager.setSpeakerphoneOn(false);
             		 }else{             			
             			audioManager.setSpeakerphoneOn(true);
             		 }   
             		 
                     call.startAudio();
                }

                @Override
                public void onCallEnded(SipAudioCall call) {                	
                	call.close();
                	audioManager.setSpeakerphoneOn(false);                	
                }
            };
          
            call = manager.makeAudioCall(me.getUriString(), botSipName+"@"+ip, listener, 80);
        }
        
        catch (Exception e) {
            Log.e(TAG, "Error when trying to close manager.", e);
            if (me != null) {
                try {
                    manager.close(me.getUriString());
                } catch (Exception ee) {
                    Log.e(TAG,"Error when trying to close manager.", ee);                  
                }
            }
            if (call != null) {
                call.close();
            }
        }
    }    
}