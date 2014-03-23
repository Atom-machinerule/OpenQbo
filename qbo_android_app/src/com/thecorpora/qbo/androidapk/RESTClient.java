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


import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.UnsupportedEncodingException;
import java.util.ArrayList;
import java.util.List;
import org.apache.http.HttpEntity;
import org.apache.http.HttpResponse;
import org.apache.http.HttpVersion;
import org.apache.http.client.ClientProtocolException;
import org.apache.http.client.HttpClient;
import org.apache.http.client.ResponseHandler;
import org.apache.http.client.entity.UrlEncodedFormEntity;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.cookie.Cookie;
import org.apache.http.impl.client.BasicResponseHandler;
import org.apache.http.impl.client.DefaultHttpClient;
import org.apache.http.impl.cookie.BasicClientCookie;
import org.apache.http.message.BasicNameValuePair;
import org.apache.http.params.BasicHttpParams;
import org.apache.http.params.CoreProtocolPNames;
import org.apache.http.params.HttpConnectionParams;
import org.apache.http.params.HttpParams;
import org.apache.http.protocol.HTTP;
import org.apache.http.util.EntityUtils;
import android.content.Context;
import android.content.SharedPreferences;
import android.util.Log;


/**
 * 
 * This class is used for the comunication with the Server inside Qbo using HTTP protocol
 * 
 * @author Daniel Cuadrado Sanchez
 *
 */
public class RESTClient {
	
	private static final String PORT = "7070";
	
	private String cabecera="http://",
	 			   ip="",
	 			   port="",	  			   	

	  			   URL_MOVE_BODY = "/teleoperation/move",
	  			   URL_MOVE_HEAD = "/teleoperation/head",	  			  
	  			   URL_SPEAK="/teleoperation/speak",
	  			   URL_MOUTH = "/teleoperation/changeMouth",

	  			   quality = "50",
	  			   widthImg = "240",
	  			   heightImg = "180",	
	
	  			   URL_RIGHT_EYE =             "/image/stream?topic=/stereo/right/image_raw&quality="+quality+"&width="+widthImg+"&height="+heightImg,		 
	  			   URL_LEFT_EYE =              "/image/stream?topic=/stereo/left/image_raw&quality="+quality+"&width="+widthImg+"&height="+heightImg, 
	  			   URL_MONOCULAR =             "/image/stream?topic=/stereo/monocolular/image_raw&quality="+quality+"&width="+widthImg+"&height="+heightImg,
	  			   URL_STEREO_SELECTOR =       "/image/stream?topic=/stereo/stereo_selector/image_raw&quality="+quality+"&width="+widthImg+"&height="+heightImg,
	  			   URL_DISPARITY =             "/image/stream?topic=/stereo/stereo_follower/disparity&quality="+quality+"&width="+widthImg+"&height="+heightImg,
	  			   URL_OBJ_TRACKING =          "/image/stream?topic=/Qbo/ObjectTraking/Viewer&quality="+quality+"&width="+widthImg+"&height="+heightImg,
	  			   URL_FACE_DETECTOR =         "/image/stream?topic=/qbo_face_tracking/viewer&quality="+quality+"&width="+widthImg+"&height="+heightImg,
	  			   URL_3D =                    "/image/stream?topic=/stereo_anaglyph&quality="+quality+"&width="+widthImg+"&height="+heightImg,	 
	  					 
	  			   URL_faceTracking_ON = "/control/face_traking/start",
	  			   URL_faceTracking_OFF = "/control/face_traking/stop",
	  			   URL_3D_ONOFF = "/teleoperation/activate_3d",
	  			   		
	  			   URL_LOGIN = "/auth/mobile_login",	  			   
	  			   
	  			   URL_IP4SIP = "/teleoperation/mobile_setIpSip",
	  		  	   URL_GET_SIP_ID = "/teleoperation/mobile_getUserSipId",
	  		       URL_GET_BOT_SIP_ID = "/teleoperation/mobile_getBotSipId",
	  		       
	  		       URL_END_CALL = "/teleoperation/mobile_endCall",
	  		       
	  		       URL_START_SIP_SERVER = "/teleoperation/mobile_startSIPServer",
	  		       
	  		       URL_STOP_SIP_SERVER = "/teleoperation/mobile_stopSIPServer",	  				   
	
	  			   TAG = "AndroidQbo/REST"; 
	
	private Context ctx;
	
	Cookie cookie=null;	
	
	public RESTClient(String ip, Context ctx){
		this.ip = ip;
		this.port = PORT;		
		this.ctx = ctx;
		
		//We check whether we have a cookie saved or not.
		String cookieFileName = "cookieFileName";		
		SharedPreferences settings = ctx.getSharedPreferences(cookieFileName, 0);
		String cookieString = settings.getString("cookie", null);
	
		this.setCookie(cookieString);
	}

	/**
	 * 
	 * Tell the robot the linear and angular speed for its movement
	 * 
	 * @param x linear speed
	 * @param y angular speed
	 */
	public void post_moveBody(double x, double y){
		String[] inputData = {"line",Double.toString(x),"angu",Double.toString(y)};	
		REST_post(cabecera+ip+":"+port+URL_MOVE_BODY, inputData);				
	}

	/**
	 * Tell the robot the pan and tilt position for its head movement. So far the pan and tilt speed is defined to 300
	 * 
	 * @param x pan position
	 * @param y tilt position
	 */
	public void post_moveHead(double x, double y){		
		String[] inputData = {"yaw",Double.toString(x),"pitch",Double.toString(y)};	
		REST_post(cabecera+ip+":"+port+URL_MOVE_HEAD, inputData);		
	}	
	
	/**
	 * Tell the robot what it is going to say
	 * 
	 * @param sentence text to speak by the robot
	 */
	public void post_say(String sentence){
		String[] inputData = {"message",sentence};
		REST_post(cabecera+ip+":"+port+URL_SPEAK, inputData);		
	}
	
	/**
	 * Tell the robot which mouth to put
	 * 
	 * @param mouth number of the mouth
	 */
	public void post_mouth(int mouth){
		String[] inputData = {"mouth",String.valueOf(mouth)};
		REST_post(cabecera+ip+":"+port+URL_MOUTH, inputData);
	}
	
	/**
	 * Tell the robot to activate the stereoscopy
	 * 
	 * @param activate a String with "on" to activate or "off" to deactivate
	 */
	public void post_activate_3D(String activate){
		String[] inputData = {"activate",activate};
		REST_post(cabecera+ip+":"+port+URL_3D_ONOFF, inputData);
	}
	
	/**
	 * 
	 * We send the user name and password parameters to the robot.
	 * 
	 * We will get true if the access is allowed, false otherwise
	 * 
	 * @param userName user name
	 * @param pwd password
	 * @param aes_string string to encrypt userName and pwd
	 * @return true if the access is allowed, false if not.
	 */
	public int post_login(String userName, String pwd, String aes_string){	
		AESCipher aes = new AESCipher();
		
		byte[] bytesUserName = {} ;
		byte[] bytesPwd ={};
		try {
			
			bytesUserName = userName.getBytes("UTF-8");		
			userName = aes.encrypt(bytesUserName, aes_string);
	
			bytesPwd = pwd.getBytes("UTF-8");
			pwd = aes.encrypt(bytesPwd, aes_string);
			
		}catch (UnsupportedEncodingException uee) {			
			uee.printStackTrace();
		}catch (Exception e) {			
			e.printStackTrace();
		}	
		
		return REST_post(cabecera+ip+":"+port+URL_LOGIN, userName,pwd);	
	}
	
		
	/**
	 * We give the IP to the robot
	 * 
	 * @param ip4Linphone The IP given by the user in the Login activity is sent to the robot in order to properly configurate Linphone
	 */
	public int post_ip4sip(String ip4Linphone){
		String[] inputData = {"ip",ip4Linphone};
		return REST_post(cabecera+ip+":"+port+URL_IP4SIP, inputData);		
	}
	
	/**
	 * We get out user name created by the SIP server. 
	 */
	public String getSipId(){
		return REST_get(cabecera+ip+":"+port+URL_GET_SIP_ID);
	}	
	
	/**
	 * We get the Linphone bot user name, so we know who we have to call
	 */
	public String getBotSipId(){
		return REST_get(cabecera+ip+":"+port+URL_GET_BOT_SIP_ID);
	}
	
	/**
	 * End call.
	 */
	public String endCall(){
		return REST_get(cabecera+ip+":"+port+URL_END_CALL);
	}	
	
	
	/**
	 * Start SIP Server
	 */
	public int startSIPServer(){		
		String[] inputData = {"ecoCancelation","false"};
		return REST_post(cabecera+ip+":"+port+URL_START_SIP_SERVER,inputData);		
	}
	
	
	/**
	 * Stop SIP Sever
	 */
	public int stopSIPServer(){
		return REST_post(cabecera+ip+":"+port+URL_STOP_SIP_SERVER,null);
	}	
	
	
	
	public String getURL_IMG(int imageType){
		
		switch(imageType){
		case 0://left_eye
			return cabecera+ip+":"+port+URL_LEFT_EYE;
			
		case 1://right eye		
			return cabecera+ip+":"+port+URL_RIGHT_EYE;
						
		case 2://monocular			
			return cabecera+ip+":"+port+URL_MONOCULAR;
			
		case 3://object_tracking			
			return cabecera+ip+":"+port+URL_STEREO_SELECTOR;
			
		case 4://face_detector
			return cabecera+ip+":"+port+URL_FACE_DETECTOR;
					
		case 5://disparity_image			
			return cabecera+ip+":"+port+URL_DISPARITY;
			
		case 6://nearest_object_tracking			
			return cabecera+ip+":"+port+URL_OBJ_TRACKING;
			
		case 7://three_d			
			return cabecera+ip+":"+port+URL_3D;
			
		}
		return "";
	}
	
		
	public void stopVideo(String url){
		url = url.replace("stream", "stop");
		REST_post(url, null);
	}

	public void setIp(String ip){
		this.ip = ip;
	}	
	public void setCmdPort(String port){
		this.port=port;
	}
	public void setImgPort(String port){
		this.port=port;
	}
	
	
	public void setImgQuality(String quality){		
		URL_RIGHT_EYE =             "/image/stream?topic=/stereo/right/image_raw&quality="+quality+"&width="+widthImg+"&height="+heightImg;	 
		URL_LEFT_EYE =              "/image/stream?topic=/stereo/left/image_raw&quality="+quality+"&width="+widthImg+"&height="+heightImg;
		URL_MONOCULAR =             "/image/stream?topic=/stereo/monocolular/image_raw&quality="+quality+"&width="+widthImg+"&height="+heightImg;
		URL_STEREO_SELECTOR =       "/image/stream?topic=/stereo/stereo_selector/image_raw&quality="+quality+"&width="+widthImg+"&height="+heightImg;
		URL_DISPARITY =             "/image/stream?topic=/stereo/stereo_follower/disparity&quality="+quality+"&width="+widthImg+"&height="+heightImg;
		URL_OBJ_TRACKING =          "/image/stream?topic=/Qbo/ObjectTraking/Viewer&quality="+quality+"&width="+widthImg+"&height="+heightImg;
		URL_FACE_DETECTOR =         "/image/stream?topic=/qbo_face_tracking/viewer&quality="+quality+"&width="+widthImg+"&height="+heightImg;
		URL_3D =                    "/image/stream?topic=/stereo_anaglyph&quality="+quality+"&width="+widthImg+"&height="+heightImg;
	}
	
	
	
	
	  ////////////////////////////////
	 //           REST             //
	////////////////////////////////
	public int REST_post(String path, String[] data) {
		try  {	
			if( hasCookie() ){ 
				
				HttpParams httpParams = new BasicHttpParams();
				httpParams.setParameter(CoreProtocolPNames.PROTOCOL_VERSION, HttpVersion.HTTP_1_1);
				DefaultHttpClient client = new DefaultHttpClient(httpParams);


				HttpPost post = new HttpPost(path);				
				
				List params = new ArrayList();
				
				if(data!=null){
					for(int i=0 ; i<= data.length / 2 ; i=i+2){
						params.add(new BasicNameValuePair(data[i], data[i+1]));
					}
				}

				UrlEncodedFormEntity ent = new UrlEncodedFormEntity(params,HTTP.UTF_8);
								
				post.setEntity(ent);			
				
				//COOKIE
				post.setHeader("Cookie", "session_id="+cookie.getValue());				

				ResponseHandler<String> responseHandler=new BasicResponseHandler();

				String responsePOST = client.execute(post,responseHandler);
			
			}else{
				Log.e(TAG," You do not have the proper COOKIE therefor this acction can not be made");
				return -1;
			}

		} catch (Exception e) {
			Log.e(TAG," Error while making a POST petition "+e);
			return -1;
		}
		
		return 0;
		
	}
	
	
	//POST to login and get the COOKIE
	private int REST_post(String path, String userName, String password) {
		try  {
			HttpParams httpParams = new BasicHttpParams();
			httpParams.setParameter(CoreProtocolPNames.PROTOCOL_VERSION, HttpVersion.HTTP_1_1);
			
			// Set the timeout in milliseconds until a connection is established.
			// The default value is zero, that means the timeout is not used. 
			int timeoutConnection = 10000;
			HttpConnectionParams.setConnectionTimeout(httpParams, timeoutConnection);
			// Set the default socket timeout (SO_TIMEOUT) 
			// in milliseconds which is the timeout for waiting for data.
			int timeoutSocket = 10000;
			HttpConnectionParams.setSoTimeout(httpParams, timeoutSocket);
			
			
			DefaultHttpClient client = new DefaultHttpClient(httpParams);
							
			HttpPost post = new HttpPost(path);
					
			List params = new ArrayList();
			params.add(new BasicNameValuePair("username", userName));
			params.add(new BasicNameValuePair("password", password));

			UrlEncodedFormEntity ent = new UrlEncodedFormEntity(params,HTTP.UTF_8);

			post.setEntity(ent);	
						
			HttpResponse responsePOST = client.execute(post);
			
			HttpEntity resEntity = responsePOST.getEntity();
			
			String responseText = EntityUtils.toString(resEntity);
			
			if( responseText.equalsIgnoreCase("error") ){
				return -1;
			};
			
	        if (resEntity != null) {
	        	resEntity.consumeContent();
	        }

	        System.out.println("Post logon cookies:");
	        List<Cookie> cookies = client.getCookieStore().getCookies();	        
	        if (cookies.isEmpty()) {
	        	Log.e(TAG,"No cookies where sent back from server");	        	    
		        this.setCookie(null);		        
	        	return -1;
	        	
	        } else {
	        	cookie = cookies.get(0);
	        	//Save cookie       		         
		        this.setCookie(cookie.getValue()); 
		        return 0;
		        
	        }  

		} catch (Exception e) {
			e.printStackTrace();
			return -1;
		}
	}
	

	/*
	 * Source: http://www.androidsnippets.com/retrieve-json-from-a-rest-web-service
	 */
	public String REST_get(String url) {
		
		if( hasCookie() ){ 
			HttpClient httpclient = new DefaultHttpClient();
			HttpGet httpget = new HttpGet(url);
			httpget.setHeader("Cookie", "session_id="+cookie.getValue());
			HttpResponse response;

			try {
				response = httpclient.execute(httpget);
//				Log.i(TAG, "Status:[" + response.getStatusLine().toString() + "]");
				HttpEntity entity = response.getEntity();

				if (entity != null) {

					InputStream instream = entity.getContent();
					String result = convertStreamToString(instream);
//					Log.i(TAG, "Result of converstion: [" + result + "]");

					instream.close();
					return result;
				}
			} catch (ClientProtocolException e) {
				Log.e(TAG, "There was a protocol based error", e);
			} catch (IOException e) {
				Log.e(TAG, "There was an IO Stream related error", e);
			}
			return null;
		}else{
            return null;
		}
	} 
	
	

	private static String convertStreamToString(InputStream is) {
		/*
		 * 
		 * Source: http://senior.ceng.metu.edu.tr/2009/praeda/2009/01/11/a-simple-restful-client-at-android/
		 * 
		 * To convert the InputStream to String we use the BufferedReader.readLine()
		 * method. We iterate until the BufferedReader return null which means
		 * there's no more data to read. Each line will appended to a StringBuilder
		 * and returned as String.
		 */
		BufferedReader reader = new BufferedReader(new InputStreamReader(is));
		StringBuilder sb = new StringBuilder();

		String line = null;
		try {
			while ((line = reader.readLine()) != null) {
				sb.append(line + "\n");
			}
		} catch (IOException e) {
			e.printStackTrace();
		} finally {
			try {
				is.close();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		return sb.toString();
	}
	
	public boolean hasCookie(){
		if (cookie!= null) return true;
					  else return false;		
	}
	
	public void setCookie(String value){
		if(value != null){
			saveSharePreferenceCookie(value);
			cookie = new BasicClientCookie("user",value);	
		}else{
			removeCookie();
		}
	}
	
	public Cookie getCookie(){
		return cookie;
	}
	
	public void removeCookie(){
		cookie = null;
		this.saveSharePreferenceCookie(null);
	}
	
	private void saveSharePreferenceCookie(String value){
        String cookieFileName = "cookieFileName";	
        SharedPreferences settings = ctx.getSharedPreferences(cookieFileName, 0);
        SharedPreferences.Editor editor = settings.edit();
        editor.putString("cookie", value);
        editor.commit(); 
	}
}
