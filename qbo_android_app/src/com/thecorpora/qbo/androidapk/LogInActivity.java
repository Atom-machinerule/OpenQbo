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
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.net.ConnectivityManager;
import android.os.AsyncTask;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.util.Log;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ProgressBar;
import android.widget.Toast;


/**
 * 
 * This activity is used just for authentication. If someone wants to access to the Qbo Robot, an user name and password have to be given and they have to match 
 * what is inside Qbo.
 *      
 * 
 * @author Daniel Cuadrado Sanchez
 *
 */
public class LogInActivity extends Activity{
 
	
	private String ip;
	private Button buttonLogin;
	private EditText editText_username, editText_password,editText_ip,editText_aes;
	private SharedPreferences prefsGet;

	private RESTClient rest;

	private Context ctx;
	
	private Boolean is3g = false,
			        isWifi = false;
	
	private ConnectivityManager connectivityManager;
	
	private ProgressBar waiting ;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.login);	

		ctx = this;

		buttonLogin = (Button) findViewById(R.id.button_login);
		buttonLogin.setOnClickListener(cliclListenerLogIn);

		editText_username = (EditText) findViewById(R.id.editText_username);
		editText_password = (EditText) findViewById(R.id.editText_password);
		editText_ip = (EditText) findViewById(R.id.editText_ip);
		editText_aes = (EditText) findViewById(R.id.editText_aes);
		
		waiting = (ProgressBar) findViewById(R.id.progressBar1);
		
		connectivityManager = (ConnectivityManager)getSystemService(this.CONNECTIVITY_SERVICE);		
	}


	@Override
	protected void onStart(){		
		super.onStart();
		
		prefsGet = PreferenceManager.getDefaultSharedPreferences(getBaseContext());		
		ip = prefsGet.getString("ip", "");
		editText_ip.setText(ip);		
		
		try{
			rest = new RESTClient(ip,this);
			rest.setCookie(null);			
		
			//If we already got a cookie, then we can go to the main activity.
			if (rest.hasCookie()){				
				Intent intent = new Intent(ctx, MainActivity.class);
				ctx.startActivity(intent);
			}

			//else we do nothing, and just wait the user to insert the data.
		}catch(Exception e){
			Log.e("Qbo-Apk","Error to access Rest Client "+e);
		}
		
	}

	private OnClickListener cliclListenerLogIn = new OnClickListener(){

		@Override
		public void onClick(View v) {
			is3g = connectivityManager.getNetworkInfo(ConnectivityManager.TYPE_MOBILE).isConnectedOrConnecting();
			isWifi = connectivityManager.getNetworkInfo(ConnectivityManager.TYPE_WIFI).isConnectedOrConnecting();
			if(is3g || isWifi){
				String userName = editText_username.getText().toString();			
				String pwd = editText_password.getText().toString();
				String ip = editText_ip.getText().toString();
			    String aes = editText_aes.getText().toString();
			    
				SharedPreferences.Editor editor = prefsGet.edit();
				editor.putString("ip", ip);
				editor.commit();

				rest.setIp(ip);		
				
				waiting.setVisibility(View.VISIBLE);
				
				Login_AsyncTask myTask = new Login_AsyncTask();
				myTask.execute(userName,pwd,aes);

			}else{
				Toast.makeText(getBaseContext(),R.string.nor_3g_neither_wifi,Toast.LENGTH_LONG).show();
			}
		}		
	};

	//Avoid back button
	@Override
	public void onBackPressed() {
		this.finish();
		return;
	}

	
	//Asynchronous task to make a HTTP petition to server, in order to check whether the user and password are allowed to come in.
	private class Login_AsyncTask extends AsyncTask<String, Void, Integer> {
	     protected Integer doInBackground(String... params) {
	    	 //We send the user name and password to the server (Qbo)
	    	 return rest.post_login(params[0], params[1],params[2]);	    	 
	     }

	     protected void onPostExecute(Integer result) {
	    	 if (result >= 0){
	    		 //Everything went smooth
	    		 Intent intent = new Intent(ctx, MainActivity.class);
	    		 ctx.startActivity(intent);
	    	 }else{
	    		 //Error
	    		 Toast.makeText(getBaseContext(),R.string.user_password_correct,Toast.LENGTH_SHORT).show();
	    	 }
	    	 waiting.setVisibility(View.GONE);
	     }
	 }
}