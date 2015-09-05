/*****************************************************************************
*[File] * GParser.CPP
*[Version] * v0.96
*[Revision Date] * 2006-09-25 *[Author] * JL Juang, jl_juang@mtk.com.tw, +886-3-567-0766 EX. 26447
*[Description] * Sample code for parsing binary data from the MTK Logger Ver. 0.99E
*[Copyright] * Copyright (C) 2005 MediaTek Incorporation. All Rights Reserved
*****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "GParser.h"

int main(int argc, char* argv[])
{
    FILE *in;
    FILE *out;
    int i;

    // print information
    printf("+-------------------------------------------------------------+\n");
    printf("| [File] GParser.EXE                                          |\n");
    printf("| [Author] JL Juang, jl_juang@mtk.com.tw, 2005-08-16          |\n");
    printf("| [Copyright] Copyright (C) 2006 MediaTek Incorporation.      |\n");
    printf("| All Rights Reserved                                         |\n");
    printf("+-------------------------------------------------------------+\n");

    if (argc < 2)
    {
        printf("USAGE: GPARSER IN_FILE_NAME [OUT_FILE]\n");
        printf("EXAMPLE: GPARSER LOG.BIN\n");
        return 0;
    }

    // open file
    in = fopen(argv[1], "rb");
    if (argv[2] != NULL)
        out = fopen(argv[2], "w+");
    else
        out = fopen("LOG.TXT", "w+");

    if (in == NULL)
    {
        printf("FAIL TO OPEN INPUT FILE !!!!\n");
        fcloseall();
        return 0;
    }

    if (out == NULL)
    {
        printf("FAIL TO OPEN INPUT FILE !!!!\n");
        fcloseall();
        return 0;
    }

    for (i=0; i<LOG_SECTOR_TOTAL/LOG_SECTOR_SIZE;i++)
    {
        printf("Parsing sector #%-3d.......Parsing\r", i);
        ParseSector(i, in, out);
        printf("Parsing sector #%-3d.......OK     \n", i);
    }

    fclose(out);
    fclose(in);

    return 0;
}

unsigned long ParseSector(int iSec, FILE *in, FILE *out)
{
    UC LogBuf[LOG_SECTOR_SIZE + 0x800];
    U4 idx = 0;
    U2 u2Size = 0;
    U4 u4Cnt = 0;
    UC iInView = 0;

    // Setting values
    U4 u4FMTREG = 0;
    U2 u2SRCNT = 0;
    U2 u2RCDMET = 0;
    U2 u2RCDMOD = 0;
    U4 u4SEC = 0;
    U4 u4DIS = 0;
    U4 u4SPD = 0;
    UC ucFSEC[32];
    U4 u4BBBB = 0;

    // flag
    UC ucSETCHG = 0;
    char strBuf[MAX_STRBUF_SIZE];

    // Read data
    if (fseek(in, (long)iSec*LOG_SECTOR_SIZE, SEEK_SET) != 0)
        return 0;
    if (fread(LogBuf, LOG_SECTOR_SIZE, 1, in) == 0)
        return 0;
    idx = 0;

    ///////////////restore setting ////////////////////////
    // STEP 1: read recording counts in this sector
    memcpy(&u2SRCNT, &LogBuf[idx], sizeof(u2SRCNT));
    idx += sizeof(u2SRCNT);

    // STEP 2: load FMT REG
    memcpy(&u4FMTREG, &LogBuf[idx], sizeof(u4FMTREG));
    idx += sizeof(u4FMTREG);

    // STEP 3: rcd mode
    memcpy(&u2RCDMOD, &LogBuf[idx], sizeof(u2RCDMOD));
    idx += sizeof(u2RCDMOD);

    // STEP 4: SEC mode
    memcpy(&u4SEC, &LogBuf[idx], sizeof(u4SEC));
    idx += sizeof(u4SEC);

    // STEP 5: DIS mode
    memcpy(&u4DIS, &LogBuf[idx], sizeof(u4DIS));
    idx += sizeof(u4DIS);

    // STEP 6: SPD mode
    memcpy(&u4SPD, &LogBuf[idx], sizeof(u4SPD));
    idx += sizeof(u4SPD);

    // STEP 7: FSEC mode
    {
        int i;
        for (i=0 ; i<32 ; i++)
            ucFSEC[i] = LogBuf[idx+i];
        idx += sizeof(ucFSEC);
    }

    // STEP 8: End pattern
//    Idx = 0x200 - sizeof(u4BBBB);
    idx = 0x200 - sizeof(u4BBBB);
    memcpy(&u4BBBB, &LogBuf[idx], sizeof(u4BBBB));
    idx += sizeof(u4BBBB);

    if (u4BBBB != 0xBBBBBBBB)
    {
        // invalid pattern
        return u4Cnt;
    }
    else
    {
        int i;
        fprintf(out, "##################################################\n");
        fprintf(out, "# SECTOR # : %-8d                                #\n", iSec);
        fprintf(out, "# SECTOR COUNT : %-8x                            #\n", u2SRCNT);
        fprintf(out, "# FORMAT REGISTER: %-8lx                         #\n", u4FMTREG );
        if (u2RCDMOD & 0x04)
        {
            fprintf(out, "# RCD METHOD : %-8s                              #\n", "STOP");
        }
        else
        {
            fprintf(out, "# RCD METHOD : %-8s                              #\n","OVP");
        }
        fprintf(out, "# RCD MODE : %-8x                                #\n", u2RCDMOD);
        fprintf(out, "# SEC MODE : %-8d                                #\n", u4SEC);
        fprintf(out, "# DIS MODE : %-8d                                #\n", u4DIS);
        fprintf(out, "# SPD MODE : %-8d                                #\n", u4SPD);
        fprintf(out, "# FSEC MODE : ");
        for (i=0; i<32 ; i++)
            fprintf(out, "%-2x ", ucFSEC[i]);
        fprintf(out, "#\n");
        fprintf(out, "##################################################\n");
    }

    ////////////////////////////////////////////////////////
    while(1)
    {
        u2Size = 0;

        // clear buffer
        strcpy(strBuf,"");

        // parse
        ucSETCHG = true;

        // --------------------------------------------
        // check pattern:
        // --------------------------------------------
        // AA AA AA AA - AA AA AA ID
        // DD DD DD DD - BB BB BB BB
        // --------------------------------------------
        // 0 ~ 6 : AA AA AA AA AA AA AA ==> intitial pattern
        //     7 : ID ==> ID of the Setting
        // 8 ~ 11: DD DD DD DD ==> Data of the setting
        // 12 ~ 15: BB BB BB BB ==> End pattern
        // --------------------------------------------

        for (int i = 0;i<16;i++)
        {
            if ((LogBuf[idx+i] != 0xAA) && i < 7)
                ucSETCHG = 0;
            if ((i>11) &&(LogBuf[idx+i] != 0xBB))
                ucSETCHG = false;
        }

        if (ucSETCHG)
        {
            if (LogBuf[idx+7] == RCD_FIELD)
            {
                memcpy(&u4FMTREG, &LogBuf[idx+8], 4);
                fprintf(out, "<CHANGE FORMAT : %08lxh >\n", u4FMTREG);
            }
            else if (LogBuf[idx+7] == BY_SEC)
            {
                memcpy(&u4SEC, &LogBuf[idx+8], 4);
                fprintf(out, "<CHANGE SEC : %08f >\n", u4SEC/10.0);
            }
            else if (LogBuf[idx+7] == BY_DIS)
            {
                memcpy(&u4DIS, &LogBuf[idx+8], 4);
                fprintf(out, "<CHANGE DIS : %08f >\n", u4DIS/10.0);
            }
            else if (LogBuf[idx+7] == BY_SPD)
            {
                memcpy(&u4SPD, &LogBuf[idx+8], 4);
                fprintf(out, "<CHANGE SPD : %08f >\n", u4SPD/10.0);
            }
            else if (LogBuf[idx+7] == RCD_METHOD)
            {
                memcpy(&u2RCDMET, &LogBuf[idx+8], 2);
                fprintf(out, "<CHANGE METHOD : %04xh >\n", u2RCDMET);
            }
            else if (LogBuf[idx+7] == LOG_STA)
            {
                memcpy(&u2RCDMOD, &LogBuf[idx+8], 2);
                fprintf(out, "<CHANGE MOD : %04xh >\n", u2RCDMOD);
            }
            idx += 16;
            continue;
        }

        if (u4FMTREG & FMT_UTC)
        {
            time_t t1;
            sprintf(strBuf+strlen(strBuf), "%10s: ", "UTC");
            memcpy(&t1, &LogBuf[idx+u2Size], sizeof(time_t));
            if (ctime(&t1) == NULL)
            {
                sprintf(strBuf+strlen(strBuf), "%10s", "NO TIME DATA\n");
                break;
            }
            else
            {
                sprintf(strBuf+strlen(strBuf), "%10s", ctime((time_t*)&LogBuf[idx+u2Size]));
            }
            u2Size += 4;
        }

        if (u4FMTREG & FMT_VAL)
        {
            sprintf(strBuf+strlen(strBuf), "%10s: ", "VAL");
            if (LogBuf[idx+u2Size] & FMT_VAL_FIX) sprintf(strBuf+strlen(strBuf), "[FIX]");
            if (LogBuf[idx+u2Size] & FMT_VAL_SPS) sprintf(strBuf+strlen(strBuf), " [SPS]");
            if (LogBuf[idx+u2Size] & FMT_VAL_DGPS) sprintf(strBuf+strlen(strBuf), " [DGPS]");
            if (LogBuf[idx+u2Size] & FMT_VAL_EST) sprintf(strBuf+strlen(strBuf), " [EST]");
            if (LogBuf[idx+u2Size] == 0x00)
                sprintf(strBuf+strlen(strBuf), "[NO FIX]");
            sprintf(strBuf+strlen(strBuf), "\n");
            u2Size += 2;
        }

        if (u4FMTREG & FMT_LAT)
        {
            double r8Tmp;
            sprintf(strBuf+strlen(strBuf), "%10s: ", "LAT");
            memcpy(&r8Tmp, &LogBuf[idx+u2Size], 8);
            sprintf(strBuf+strlen(strBuf), "%-9.6f", r8Tmp);
            sprintf(strBuf+strlen(strBuf), "\n");
            u2Size += 8;
        }

        if (u4FMTREG & FMT_LON)
        {
            double r8Tmp;
            sprintf(strBuf+strlen(strBuf), "%10s: ", "LON");
            memcpy(&r8Tmp, &LogBuf[idx+u2Size], 8);
            sprintf(strBuf+strlen(strBuf), "%-9.6f", r8Tmp);
            sprintf(strBuf+strlen(strBuf), "\n");
            u2Size += 8;
        }

        if (u4FMTREG & FMT_HGT)
        {
            float r4Tmp;
            sprintf(strBuf+strlen(strBuf), "%10s: ", "HGT");
            memcpy(&r4Tmp, &LogBuf[idx+u2Size], 4);
            sprintf(strBuf+strlen(strBuf), "%-9.2f", r4Tmp);
            sprintf(strBuf+strlen(strBuf), "\n");
            u2Size += 4;
        }

        if (u4FMTREG & FMT_SPD)
        {
            float r4Tmp;
            sprintf(strBuf+strlen(strBuf), "%10s: ", "SPD");
            memcpy(&r4Tmp, &LogBuf[idx+u2Size], 4);
            sprintf(strBuf+strlen(strBuf), "%-9.2f", r4Tmp);
            sprintf(strBuf+strlen(strBuf), "\n");
            u2Size += 4;
        }

        if (u4FMTREG & FMT_TRK)
        {
            float r4Tmp;
            sprintf(strBuf+strlen(strBuf), "%10s: ", "TRK");
            memcpy(&r4Tmp, &LogBuf[idx+u2Size], 4);
            sprintf(strBuf+strlen(strBuf), "%-9.2f", r4Tmp);
            sprintf(strBuf+strlen(strBuf), "\n");
            u2Size += 4;
        }

        if (u4FMTREG & FMT_DSTA)
        {
            U2 u2Tmp;
            sprintf(strBuf+strlen(strBuf), "%10s: ", "DSTA");
            memcpy(&u2Tmp, &LogBuf[idx+u2Size], 2);
            sprintf(strBuf+strlen(strBuf), "%x", u2Tmp);
            sprintf(strBuf+strlen(strBuf), "\n");
            u2Size += 2;
        }

        if (u4FMTREG & FMT_DAGE)
        {
            float r4Tmp;
            sprintf(strBuf+strlen(strBuf), "%10s: ","DAGE");
            memcpy(&r4Tmp, &LogBuf[idx+u2Size], 4);
            sprintf(strBuf+strlen(strBuf), "%-9.2f", r4Tmp);
            sprintf(strBuf+strlen(strBuf), "\n");
            u2Size += 4;
        }

        if (u4FMTREG & FMT_PDOP)
        {
            U2 u2Tmp;
            sprintf(strBuf+strlen(strBuf), "%10s: ", "PDOP");
            memcpy(&u2Tmp, &LogBuf[idx+u2Size], 2);
            sprintf(strBuf+strlen(strBuf), "%-9.2f", (float)u2Tmp/100.0);
            sprintf(strBuf+strlen(strBuf), "\n");
            u2Size += 2;
        }

        if (u4FMTREG & FMT_HDOP)
        {
            U2 u2Tmp;
            sprintf(strBuf+strlen(strBuf), "%10s: ", "HDOP");
            memcpy(&u2Tmp, &LogBuf[idx+u2Size], 2);
            sprintf(strBuf+strlen(strBuf), "%-9.2f", (float)u2Tmp/100.0);
            sprintf(strBuf+strlen(strBuf), "\n");
            u2Size += 2;
        }

        if (u4FMTREG & FMT_VDOP)
        {
            U2 u2Tmp;
            sprintf(strBuf+strlen(strBuf), "%10s: ","VDOP");
            memcpy(&u2Tmp, &LogBuf[idx+u2Size], 2);
            sprintf(strBuf+strlen(strBuf), "%-9.2f", (float)u2Tmp/100.0);
            sprintf(strBuf+strlen(strBuf), "\n");
            u2Size += 2;
        }

        if (u4FMTREG & FMT_NSAT)
        {
            sprintf(strBuf+strlen(strBuf), "%10s: ", "IN_VIEW");
            iInView =LogBuf[idx+u2Size];
            sprintf(strBuf+strlen(strBuf), "%d\n", iInView);

            sprintf(strBuf+strlen(strBuf), "%10s: ", "IN_USE");
            sprintf(strBuf+strlen(strBuf), "%d", LogBuf[idx+u2Size+1]);
            sprintf(strBuf+strlen(strBuf), "\n");
            u2Size += 2;
        }

//////////////////////////////

        if (u4FMTREG & FMT_SID)
        {
            iInView = LogBuf[idx+u2Size+2];
            if (iInView == 0xff)
            {
                iInView = 0;
            }
            for (int i=0;i<iInView;i++)
            {
                // SID part
                unsigned char uSID = LogBuf[idx+u2Size];
                sprintf(strBuf+strlen(strBuf), "--------------------------------\n");
                sprintf(strBuf+strlen(strBuf), " %10s#", "SID");
                sprintf(strBuf+strlen(strBuf), " %02d", uSID);
                // in use check
                if (LogBuf[idx+u2Size+1] & 0x01)
                    sprintf(strBuf+strlen(strBuf), "%s"," [IN_USE]");
                sprintf(strBuf+strlen(strBuf), "\n");
                u2Size += 4;

                // ELE
                if (u4FMTREG & FMT_ELE)
                {
                    signed char i1Tmp;
                    sprintf(strBuf+strlen(strBuf), " %10s: ", "ELE");
                    memcpy(&i1Tmp, &LogBuf[idx+u2Size], 1);
                    sprintf(strBuf+strlen(strBuf), "%02d", i1Tmp);
                    sprintf(strBuf+strlen(strBuf), "\n");
                    u2Size += 2;
                }

                //AZI
                if (u4FMTREG & FMT_AZI )
                {
                    short i2Tmp;
                    sprintf(strBuf+strlen(strBuf), " %10s: ", "AZI");
                    memcpy(&i2Tmp, &LogBuf[idx+u2Size], 2);
                    sprintf(strBuf+strlen(strBuf), "%02d", i2Tmp);
                    sprintf(strBuf+strlen(strBuf), "\n");
                    u2Size += 2;
                }

                //SNR
                if (u4FMTREG & FMT_SNR)
                {
                    unsigned short u2Tmp;
                    sprintf(strBuf+strlen(strBuf), " %10s: ","SNR");
                    memcpy(&u2Tmp, &LogBuf[idx+u2Size], 2);
                    sprintf(strBuf+strlen(strBuf), "%02d", u2Tmp);
                    sprintf(strBuf+strlen(strBuf), "\n");
                    u2Size += 2;
                }
            }

            if (iInView == 0)
            {
                // Empty SID
                sprintf(strBuf+strlen(strBuf), "%10s: ", "SID");
                sprintf(strBuf+strlen(strBuf), "%s", "NO SAT IN VIEW");
                sprintf(strBuf+strlen(strBuf), "\n");
                u2Size+= 4;
            }
        }

        if (u4FMTREG & FMT_RCR)
        {
            unsigned short u2RCR;
            sprintf(strBuf+strlen(strBuf), "%10s: ","RCR");
            memcpy(&u2RCR, &LogBuf[idx+u2Size], 2);
            sprintf(strBuf+strlen(strBuf), "%x",u2RCR);
            if (u2RCR & FMT_RCR_SEC)
            {
                sprintf(strBuf+strlen(strBuf), " [SEC]");
            }
            if (u2RCR & FMT_RCR_SPD)
            {
                sprintf(strBuf+strlen(strBuf), " [SPD]");
            }
            if (u2RCR & FMT_RCR_DIS)
            {
                sprintf(strBuf+strlen(strBuf), " [DIS]");
            }
            if (u2RCR & FMT_RCR_LN)
            {
                sprintf(strBuf+strlen(strBuf), " [BTN]");
            }
            sprintf(strBuf+strlen(strBuf), "\n");
            u2Size += 2;
        }
        if (u4FMTREG & FMT_MS)
        {
            unsigned short u2Tmp;
            sprintf(strBuf+strlen(strBuf), "%10s: ","MS");
            memcpy(&u2Tmp, &LogBuf[idx+u2Size], 2);
            sprintf(strBuf+strlen(strBuf), "%d", u2Tmp);
            sprintf(strBuf+strlen(strBuf), "\n");
            u2Size += 2;
        }

        //check sum
        u2Size += 2;

/////////////////////////////////

        if ((idx+u2Size) > LOG_SECTOR_SIZE)
        {
            break;
        }
        else
        {
            int i;
            for (i=0;i<u2Size;i++)
            {
                if (LogBuf[idx+i] != 0xff)
                    break;
            }
            if (i==u2Size)
            {
                break;
            }
            if (Checksum_Verify())
            {
                u4Cnt++;
                fprintf(out, "(%d)===================================================\n", u4Cnt);
                fprintf(out, "%s",strBuf);
            }
            idx += u2Size;
        }
        if (idx >= LOG_SECTOR_SIZE)
            break;
    }

    return u4Cnt;
}
