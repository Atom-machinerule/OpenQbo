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

import java.io.ByteArrayOutputStream;
import java.security.SecureRandom;

import org.bouncycastle.crypto.BufferedBlockCipher;
import org.bouncycastle.crypto.PBEParametersGenerator;
import org.bouncycastle.crypto.engines.AESEngine;
import org.bouncycastle.crypto.modes.CBCBlockCipher;
import org.bouncycastle.crypto.paddings.PaddedBufferedBlockCipher;

import org.bouncycastle.crypto.generators.OpenSSLPBEParametersGenerator;
import org.bouncycastle.crypto.params.KeyParameter;
import org.bouncycastle.crypto.params.ParametersWithIV;
import android.util.Base64;


public class AESCipher {

	//From https://forums.oracle.com/forums/thread.jspa?threadID=1528523

	// configData is the data that is to be encrypted
	// key is the key from which actual encryption key is to be generated

	public String encrypt(byte configData[], String key) throws Exception
	{
		byte []encryptedConfigData = null;
		AESEngine blockCipher = new AESEngine();
		blockCipher.reset();
		CBCBlockCipher cbcCipher = new CBCBlockCipher(blockCipher);
		BufferedBlockCipher bbc = new PaddedBufferedBlockCipher(cbcCipher);

		byte[] salt = new byte[8];
		SecureRandom secure = new SecureRandom();
		secure.nextBytes(salt);

		//intialising in the encryption mode with Key and IV
		bbc.init(true,getKeyParamWithIv(key, salt));
		byte []encryptedData = new byte[bbc.getOutputSize(configData.length)];

		//process array of bytes
		int noOfBytes = bbc.processBytes(configData,0,configData.length,encryptedData,0);

		//process the last block in the buffer
		bbc.doFinal(encryptedData, noOfBytes);

		ByteArrayOutputStream bos = new ByteArrayOutputStream();
		//writing encrypted data along with the salt in the format readable by open ssl api
		bos.write("Salted__".getBytes());
		bos.write(salt);
		bos.write(encryptedData);		
		encryptedConfigData = bos.toByteArray();		
		bos.close();

//		return encryptedConfigData;
		return Base64.encodeToString(encryptedConfigData, Base64.DEFAULT);		
		
	}

	private ParametersWithIV getKeyParamWithIv(String keyphrase, byte[] salt) {
		int iterationCount = 1;
		//creating generator for PBE derived keys and ivs as used by open ssl
		PBEParametersGenerator generator = new OpenSSLPBEParametersGenerator();

		//intialse the PBE generator with password, salt and iteration count
		generator.init(PBEParametersGenerator.PKCS5PasswordToBytes(keyphrase.toCharArray()), salt, iterationCount);

		//Generate a key with initialisation vector parameter derived from the password, salt and iteration count
		ParametersWithIV paramWithIv = (ParametersWithIV)generator.generateDerivedParameters(256,128);
		KeyParameter keyParam = (KeyParameter) paramWithIv.getParameters();

		return paramWithIv;
	}

	

	

}
