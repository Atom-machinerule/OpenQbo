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

import android.content.SharedPreferences;
import android.os.Bundle;
import android.preference.Preference;
import android.preference.PreferenceManager;
import android.preference.Preference.OnPreferenceChangeListener;
import android.preference.PreferenceActivity;
import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;


public class QboPreferences extends PreferenceActivity implements OnSeekBarChangeListener{
	Preference portSIPOption;
	Preference delayOption; 
	Preference ipOption;
	Preference imageTypeOption, languageOption, linearSpeedOpt, angularSpeedOpt,linearSensivity, angularSensivity, quality;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		// Load the preferences from an XML resource
		addPreferencesFromResource(R.xml.preferences);

		// Checking the values inserted by user	
		ipOption = (Preference) findPreference("ip"); 
		ipOption.setOnPreferenceChangeListener(IPChanged);	

		quality = (Preference) findPreference("quality"); 
		quality.setOnPreferenceChangeListener(qualityChanged);	

	}

	@Override
	protected void onResume(){
		super.onResume();
		SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(getBaseContext()); 

		ipOption.setSummary(prefs.getString("ip",getString( R.string.preferences_ip)));

		quality.setSummary(prefs.getString("quality", getString(R.string.preferences_quality)));		

		String a = prefs.getString("quality", "-1");
		setSummaryForQuality(a);
	}


	private OnPreferenceChangeListener IPChanged  = new OnPreferenceChangeListener(){
		@Override
		public boolean onPreferenceChange(Preference preference, Object newValue) {
			preference.setSummary((CharSequence) newValue);
			return true;
		}
	};


	private OnPreferenceChangeListener qualityChanged  = new OnPreferenceChangeListener(){
		@Override
		public boolean onPreferenceChange(Preference preference, Object newValue) {
			setSummaryForQuality((String)newValue);
			return true;		
		}
	};


	private void setSummaryForQuality(String newValue){			
		if( newValue.equalsIgnoreCase(getString(R.string.quality_top)))	quality.setSummary(R.string.preferences_top_quality_label);
		else if(newValue.equalsIgnoreCase(getString(R.string.quality_high)))quality.setSummary(R.string.preferences_high_quality_label);
		else if(newValue.equalsIgnoreCase(getString(R.string.quality_normal)))	quality.setSummary(R.string.preferences_normal_quality_label);
		else if(newValue.equalsIgnoreCase(getString(R.string.quality_medium)))	quality.setSummary(R.string.preferences_medium_quality_label);
		else if(newValue.equalsIgnoreCase(getString(R.string.quality_low)))	quality.setSummary(R.string.preferences_low_quality_label);	
	}



	public void setSummaryForImageType(int i){
		switch(i){
		case 0:
			imageTypeOption.setSummary(this.getString(R.string.left_eye));
			break;
		case 1:			
			imageTypeOption.setSummary(this.getString(R.string.right_eye));
			break;
		case 2:			
			imageTypeOption.setSummary(this.getString(R.string.monocular));
			break;
		case 3:			
			imageTypeOption.setSummary(this.getString(R.string.object_tracking));
			break;
		case 4:
			imageTypeOption.setSummary(this.getString(R.string.face_detector));
			break;			
		case 5:			
			imageTypeOption.setSummary(this.getString(R.string.disparity_image));
			break;
		case 6:			
			imageTypeOption.setSummary(this.getString(R.string.nearest_object_tracking));
			break;
		case 7:			
			imageTypeOption.setSummary(this.getString(R.string.three_d));
			break;
		}
	}


	@Override
	public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
		seekBar.setProgress(progress);
	}

	@Override
	public void onStartTrackingTouch(SeekBar seekBar) {
		// TODO Auto-generated method stub		
	}

	@Override
	public void onStopTrackingTouch(SeekBar seekBar) {
		// TODO Auto-generated method stub		
	}	



}