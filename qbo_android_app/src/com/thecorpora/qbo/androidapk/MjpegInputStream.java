 /*
 * Software License Agreement (GPLv2 License)
 * 
 * Copyright (c) 2010, Robert Bosch LLC.
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



import java.io.BufferedInputStream;
import java.io.ByteArrayInputStream;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStream;

import java.net.URI;
import java.util.Properties;

import org.apache.http.HttpResponse;
import org.apache.http.client.ClientProtocolException;
import org.apache.http.client.CookieStore;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.cookie.Cookie;
import org.apache.http.impl.client.BasicCookieStore;
import org.apache.http.impl.client.DefaultHttpClient;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.util.Log;

public class MjpegInputStream extends DataInputStream {
    private final byte[] SOI_MARKER = { (byte) 0xFF, (byte) 0xD8 };
    private final byte[] EOF_MARKER = { (byte) 0xFF, (byte) 0xD9 };
    private final String CONTENT_LENGTH = "Content-Length";
    private final static int HEADER_MAX_LENGTH = 100;
    private final static int FRAME_MAX_LENGTH = 40000 + HEADER_MAX_LENGTH;
    private int mContentLength = -1;
	
    public static MjpegInputStream read(String url, Cookie cookie) {
        HttpResponse res;
        DefaultHttpClient httpclient = new DefaultHttpClient();	     

        HttpGet get = new HttpGet(url);  
        Log.d("COOKIE",cookie.getValue());
        get.setHeader("Cookie", "session_id="+cookie.getValue());
        
        try {            	
            res = httpclient.execute(get);
            return new MjpegInputStream(res.getEntity().getContent());				
        } catch (ClientProtocolException e) {
        } catch (IOException e) {}
        return null;
    }
	
    public MjpegInputStream(InputStream in) { super(new BufferedInputStream(in, FRAME_MAX_LENGTH)); }
	
    private int getEndOfSeqeunce(DataInputStream in, byte[] sequence) throws IOException {
        int seqIndex = 0;
        byte c;
        for(int i=0; i < FRAME_MAX_LENGTH; i++) {
            c = (byte) in.readUnsignedByte();
            if(c == sequence[seqIndex]) {
                seqIndex++;
                if(seqIndex == sequence.length) return i + 1;
            } else seqIndex = 0;
        }
        return -1;
    }
	
    private int getStartOfSequence(DataInputStream in, byte[] sequence) throws IOException {
        int end = getEndOfSeqeunce(in, sequence);
        return (end < 0) ? (-1) : (end - sequence.length);
    }

    private int parseContentLength(byte[] headerBytes) throws IOException, NumberFormatException {
        ByteArrayInputStream headerIn = new ByteArrayInputStream(headerBytes);
        Properties props = new Properties();
        props.load(headerIn);
        return Integer.parseInt(props.getProperty(CONTENT_LENGTH));
    }	

    public Bitmap readMjpegFrame() throws IOException {
        mark(FRAME_MAX_LENGTH);
        int headerLen = getStartOfSequence(this, SOI_MARKER);
        reset();
        byte[] header = new byte[headerLen];
        readFully(header);
        try {
            mContentLength = parseContentLength(header);
        } catch (NumberFormatException nfe) { 
            mContentLength = getEndOfSeqeunce(this, EOF_MARKER); 
        }
        reset();
        byte[] frameData = new byte[mContentLength];
        skipBytes(headerLen);
        readFully(frameData);
        return BitmapFactory.decodeStream(new ByteArrayInputStream(frameData));
    }
}