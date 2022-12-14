/*
 * Copyright (C) 2015-2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

package com.swiftnav.sbp.ssr;

import java.math.BigInteger;

import com.swiftnav.sbp.SBPMessage;
import com.swiftnav.sbp.SBPBinaryException;
import com.swiftnav.sbp.SBPStruct;
import com.swiftnav.sbp.gnss.*;

import org.json.JSONObject;
import org.json.JSONArray;


/** SBP class for message MSG_SSR_GRIDDED_CORRECTION (0x05F0).
 *
 * You can have MSG_SSR_GRIDDED_CORRECTION inherent its fields directly from
 * an inherited SBP object, or construct it inline using a dict of its
 * fields.
 *
 * STEC residuals are per space vehicle, tropo is not. */

public class MsgSsrGriddedCorrection extends SBPMessage {
    public static final int TYPE = 0x05F0;

    
    /** Header of a Gridded Correction message */
    public GriddedCorrectionHeader header;
    
    /** Tropo and STEC residuals for the given grid point */
    public GridElement element;
    

    public MsgSsrGriddedCorrection (int sender) { super(sender, TYPE); }
    public MsgSsrGriddedCorrection () { super(TYPE); }
    public MsgSsrGriddedCorrection (SBPMessage msg) throws SBPBinaryException {
        super(msg);
        assert msg.type != TYPE;
    }

    @Override
    protected void parse(Parser parser) throws SBPBinaryException {
        /* Parse fields from binary */
        header = new GriddedCorrectionHeader().parse(parser);
        element = new GridElement().parse(parser);
    }

    @Override
    protected void build(Builder builder) {
        header.build(builder);
        element.build(builder);
    }

    @Override
    public JSONObject toJSON() {
        JSONObject obj = super.toJSON();
        obj.put("header", header.toJSON());
        obj.put("element", element.toJSON());
        return obj;
    }
}