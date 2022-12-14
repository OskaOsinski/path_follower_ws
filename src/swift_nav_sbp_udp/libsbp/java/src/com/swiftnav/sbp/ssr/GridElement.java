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
import com.swiftnav.sbp.SBPStruct;

public class GridElement extends SBPStruct {
    
    /** index of the grid point */
    public int index;
    
    /** Wet and Hydrostatic Vertical Delay */
    public TroposphericDelayCorrection tropo_delay_correction;
    
    /** STEC Residual for the given space vehicle */
    public STECResidual[] STEC_residuals;
    

    public GridElement () {}

    @Override
    public GridElement parse(SBPMessage.Parser parser) throws SBPBinaryException {
        /* Parse fields from binary */
        index = parser.getU16();
        tropo_delay_correction = new TroposphericDelayCorrection().parse(parser);
        STEC_residuals = parser.getArray(STECResidual.class);
        return this;
    }

    @Override
    public void build(SBPMessage.Builder builder) {
        /* Build fields into binary */
        builder.putU16(index);
        tropo_delay_correction.build(builder);
        builder.putArray(STEC_residuals);
    }

    @Override
    public JSONObject toJSON() {
        JSONObject obj = new JSONObject();
        obj.put("index", index);
        obj.put("tropo_delay_correction", tropo_delay_correction.toJSON());
        obj.put("STEC_residuals", SBPStruct.toJSONArray(STEC_residuals));
        return obj;
    }
}