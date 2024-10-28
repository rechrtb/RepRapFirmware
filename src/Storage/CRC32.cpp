// CRC-32 generation class methods

#include "CRC32.h"

#if USE_SAME5x_HARDWARE_CRC

# include <RTOSIface/RTOSIface.h>

// Reverse the order of bits in a 32-bit word
static inline uint32_t Reflect(uint32_t v) noexcept
{
	uint32_t rslt;
	asm("rbit %0,%1" : "=r" (rslt) : "r" (v));
	return rslt;
}

#endif

constexpr uint32_t CRC_32_TAB[256] =
{
	// CRC polynomial 0xedb88320
	0x00000000u, 0x77073096u, 0xee0e612cu, 0x990951bau, 0x076dc419u, 0x706af48fu,
	0xe963a535u, 0x9e6495a3u, 0x0edb8832u, 0x79dcb8a4u, 0xe0d5e91eu, 0x97d2d988u,
	0x09b64c2bu, 0x7eb17cbdu, 0xe7b82d07u, 0x90bf1d91u, 0x1db71064u, 0x6ab020f2u,
	0xf3b97148u, 0x84be41deu, 0x1adad47du, 0x6ddde4ebu, 0xf4d4b551u, 0x83d385c7u,
	0x136c9856u, 0x646ba8c0u, 0xfd62f97au, 0x8a65c9ecu, 0x14015c4fu, 0x63066cd9u,
	0xfa0f3d63u, 0x8d080df5u, 0x3b6e20c8u, 0x4c69105eu, 0xd56041e4u, 0xa2677172u,
	0x3c03e4d1u, 0x4b04d447u, 0xd20d85fdu, 0xa50ab56bu, 0x35b5a8fau, 0x42b2986cu,
	0xdbbbc9d6u, 0xacbcf940u, 0x32d86ce3u, 0x45df5c75u, 0xdcd60dcfu, 0xabd13d59u,
	0x26d930acu, 0x51de003au, 0xc8d75180u, 0xbfd06116u, 0x21b4f4b5u, 0x56b3c423u,
	0xcfba9599u, 0xb8bda50fu, 0x2802b89eu, 0x5f058808u, 0xc60cd9b2u, 0xb10be924u,
	0x2f6f7c87u, 0x58684c11u, 0xc1611dabu, 0xb6662d3du, 0x76dc4190u, 0x01db7106u,
	0x98d220bcu, 0xefd5102au, 0x71b18589u, 0x06b6b51fu, 0x9fbfe4a5u, 0xe8b8d433u,
	0x7807c9a2u, 0x0f00f934u, 0x9609a88eu, 0xe10e9818u, 0x7f6a0dbbu, 0x086d3d2du,
	0x91646c97u, 0xe6635c01u, 0x6b6b51f4u, 0x1c6c6162u, 0x856530d8u, 0xf262004eu,
	0x6c0695edu, 0x1b01a57bu, 0x8208f4c1u, 0xf50fc457u, 0x65b0d9c6u, 0x12b7e950u,
	0x8bbeb8eau, 0xfcb9887cu, 0x62dd1ddfu, 0x15da2d49u, 0x8cd37cf3u, 0xfbd44c65u,
	0x4db26158u, 0x3ab551ceu, 0xa3bc0074u, 0xd4bb30e2u, 0x4adfa541u, 0x3dd895d7u,
	0xa4d1c46du, 0xd3d6f4fbu, 0x4369e96au, 0x346ed9fcu, 0xad678846u, 0xda60b8d0u,
	0x44042d73u, 0x33031de5u, 0xaa0a4c5fu, 0xdd0d7cc9u, 0x5005713cu, 0x270241aau,
	0xbe0b1010u, 0xc90c2086u, 0x5768b525u, 0x206f85b3u, 0xb966d409u, 0xce61e49fu,
	0x5edef90eu, 0x29d9c998u, 0xb0d09822u, 0xc7d7a8b4u, 0x59b33d17u, 0x2eb40d81u,
	0xb7bd5c3bu, 0xc0ba6cadu, 0xedb88320u, 0x9abfb3b6u, 0x03b6e20cu, 0x74b1d29au,
	0xead54739u, 0x9dd277afu, 0x04db2615u, 0x73dc1683u, 0xe3630b12u, 0x94643b84u,
	0x0d6d6a3eu, 0x7a6a5aa8u, 0xe40ecf0bu, 0x9309ff9du, 0x0a00ae27u, 0x7d079eb1u,
	0xf00f9344u, 0x8708a3d2u, 0x1e01f268u, 0x6906c2feu, 0xf762575du, 0x806567cbu,
	0x196c3671u, 0x6e6b06e7u, 0xfed41b76u, 0x89d32be0u, 0x10da7a5au, 0x67dd4accu,
	0xf9b9df6fu, 0x8ebeeff9u, 0x17b7be43u, 0x60b08ed5u, 0xd6d6a3e8u, 0xa1d1937eu,
	0x38d8c2c4u, 0x4fdff252u, 0xd1bb67f1u, 0xa6bc5767u, 0x3fb506ddu, 0x48b2364bu,
	0xd80d2bdau, 0xaf0a1b4cu, 0x36034af6u, 0x41047a60u, 0xdf60efc3u, 0xa867df55u,
	0x316e8eefu, 0x4669be79u, 0xcb61b38cu, 0xbc66831au, 0x256fd2a0u, 0x5268e236u,
	0xcc0c7795u, 0xbb0b4703u, 0x220216b9u, 0x5505262fu, 0xc5ba3bbeu, 0xb2bd0b28u,
	0x2bb45a92u, 0x5cb36a04u, 0xc2d7ffa7u, 0xb5d0cf31u, 0x2cd99e8bu, 0x5bdeae1du,
	0x9b64c2b0u, 0xec63f226u, 0x756aa39cu, 0x026d930au, 0x9c0906a9u, 0xeb0e363fu,
	0x72076785u, 0x05005713u, 0x95bf4a82u, 0xe2b87a14u, 0x7bb12baeu, 0x0cb61b38u,
	0x92d28e9bu, 0xe5d5be0du, 0x7cdcefb7u, 0x0bdbdf21u, 0x86d3d2d4u, 0xf1d4e242u,
	0x68ddb3f8u, 0x1fda836eu, 0x81be16cdu, 0xf6b9265bu, 0x6fb077e1u, 0x18b74777u,
	0x88085ae6u, 0xff0f6a70u, 0x66063bcau, 0x11010b5cu, 0x8f659effu, 0xf862ae69u,
	0x616bffd3u, 0x166ccf45u, 0xa00ae278u, 0xd70dd2eeu, 0x4e048354u, 0x3903b3c2u,
	0xa7672661u, 0xd06016f7u, 0x4969474du, 0x3e6e77dbu, 0xaed16a4au, 0xd9d65adcu,
	0x40df0b66u, 0x37d83bf0u, 0xa9bcae53u, 0xdebb9ec5u, 0x47b2cf7fu, 0x30b5ffe9u,
	0xbdbdf21cu, 0xcabac28au, 0x53b39330u, 0x24b4a3a6u, 0xbad03605u, 0xcdd70693u,
	0x54de5729u, 0x23d967bfu, 0xb3667a2eu, 0xc4614ab8u, 0x5d681b02u, 0x2a6f2b94u,
	0xb40bbe37u, 0xc30c8ea1u, 0x5a05df1bu, 0x2d02ef8du
};

# if SAME70 || (SAME5x && !USE_SAME5x_HARDWARE_CRC)

// On SAME70 we have more flash memory available, so use 4K table instead of 1K and the faster slicing-by-4 algorithm
constexpr uint32_t CRC_32_TAB1[256] =
{
	0x00000000u, 0x191B3141u, 0x32366282u, 0x2B2D53C3u, 0x646CC504u, 0x7D77F445u, 0x565AA786u, 0x4F4196C7u,
	0xC8D98A08u, 0xD1C2BB49u, 0xFAEFE88Au, 0xE3F4D9CBu, 0xACB54F0Cu, 0xB5AE7E4Du, 0x9E832D8Eu, 0x87981CCFu,
	0x4AC21251u, 0x53D92310u, 0x78F470D3u, 0x61EF4192u, 0x2EAED755u, 0x37B5E614u, 0x1C98B5D7u, 0x05838496u,
	0x821B9859u, 0x9B00A918u, 0xB02DFADBu, 0xA936CB9Au, 0xE6775D5Du, 0xFF6C6C1Cu, 0xD4413FDFu, 0xCD5A0E9Eu,
	0x958424A2u, 0x8C9F15E3u, 0xA7B24620u, 0xBEA97761u, 0xF1E8E1A6u, 0xE8F3D0E7u, 0xC3DE8324u, 0xDAC5B265u,
	0x5D5DAEAAu, 0x44469FEBu, 0x6F6BCC28u, 0x7670FD69u, 0x39316BAEu, 0x202A5AEFu, 0x0B07092Cu, 0x121C386Du,
	0xDF4636F3u, 0xC65D07B2u, 0xED705471u, 0xF46B6530u, 0xBB2AF3F7u, 0xA231C2B6u, 0x891C9175u, 0x9007A034u,
	0x179FBCFBu, 0x0E848DBAu, 0x25A9DE79u, 0x3CB2EF38u, 0x73F379FFu, 0x6AE848BEu, 0x41C51B7Du, 0x58DE2A3Cu,
	0xF0794F05u, 0xE9627E44u, 0xC24F2D87u, 0xDB541CC6u, 0x94158A01u, 0x8D0EBB40u, 0xA623E883u, 0xBF38D9C2u,
	0x38A0C50Du, 0x21BBF44Cu, 0x0A96A78Fu, 0x138D96CEu, 0x5CCC0009u, 0x45D73148u, 0x6EFA628Bu, 0x77E153CAu,
	0xBABB5D54u, 0xA3A06C15u, 0x888D3FD6u, 0x91960E97u, 0xDED79850u, 0xC7CCA911u, 0xECE1FAD2u, 0xF5FACB93u,
	0x7262D75Cu, 0x6B79E61Du, 0x4054B5DEu, 0x594F849Fu, 0x160E1258u, 0x0F152319u, 0x243870DAu, 0x3D23419Bu,
	0x65FD6BA7u, 0x7CE65AE6u, 0x57CB0925u, 0x4ED03864u, 0x0191AEA3u, 0x188A9FE2u, 0x33A7CC21u, 0x2ABCFD60u,
	0xAD24E1AFu, 0xB43FD0EEu, 0x9F12832Du, 0x8609B26Cu, 0xC94824ABu, 0xD05315EAu, 0xFB7E4629u, 0xE2657768u,
	0x2F3F79F6u, 0x362448B7u, 0x1D091B74u, 0x04122A35u, 0x4B53BCF2u, 0x52488DB3u, 0x7965DE70u, 0x607EEF31u,
	0xE7E6F3FEu, 0xFEFDC2BFu, 0xD5D0917Cu, 0xCCCBA03Du, 0x838A36FAu, 0x9A9107BBu, 0xB1BC5478u, 0xA8A76539u,
	0x3B83984Bu, 0x2298A90Au, 0x09B5FAC9u, 0x10AECB88u, 0x5FEF5D4Fu, 0x46F46C0Eu, 0x6DD93FCDu, 0x74C20E8Cu,
	0xF35A1243u, 0xEA412302u, 0xC16C70C1u, 0xD8774180u, 0x9736D747u, 0x8E2DE606u, 0xA500B5C5u, 0xBC1B8484u,
	0x71418A1Au, 0x685ABB5Bu, 0x4377E898u, 0x5A6CD9D9u, 0x152D4F1Eu, 0x0C367E5Fu, 0x271B2D9Cu, 0x3E001CDDu,
	0xB9980012u, 0xA0833153u, 0x8BAE6290u, 0x92B553D1u, 0xDDF4C516u, 0xC4EFF457u, 0xEFC2A794u, 0xF6D996D5,
	0xAE07BCE9u, 0xB71C8DA8u, 0x9C31DE6Bu, 0x852AEF2Au, 0xCA6B79EDu, 0xD37048ACu, 0xF85D1B6Fu, 0xE1462A2E,
	0x66DE36E1u, 0x7FC507A0u, 0x54E85463u, 0x4DF36522u, 0x02B2F3E5u, 0x1BA9C2A4u, 0x30849167u, 0x299FA026,
	0xE4C5AEB8u, 0xFDDE9FF9u, 0xD6F3CC3Au, 0xCFE8FD7Bu, 0x80A96BBCu, 0x99B25AFDu, 0xB29F093Eu, 0xAB84387F,
	0x2C1C24B0u, 0x350715F1u, 0x1E2A4632u, 0x07317773u, 0x4870E1B4u, 0x516BD0F5u, 0x7A468336u, 0x635DB277,
	0xCBFAD74Eu, 0xD2E1E60Fu, 0xF9CCB5CCu, 0xE0D7848Du, 0xAF96124Au, 0xB68D230Bu, 0x9DA070C8u, 0x84BB4189,
	0x03235D46u, 0x1A386C07u, 0x31153FC4u, 0x280E0E85u, 0x674F9842u, 0x7E54A903u, 0x5579FAC0u, 0x4C62CB81,
	0x8138C51Fu, 0x9823F45Eu, 0xB30EA79Du, 0xAA1596DCu, 0xE554001Bu, 0xFC4F315Au, 0xD7626299u, 0xCE7953D8,
	0x49E14F17u, 0x50FA7E56u, 0x7BD72D95u, 0x62CC1CD4u, 0x2D8D8A13u, 0x3496BB52u, 0x1FBBE891u, 0x06A0D9D0,
	0x5E7EF3ECu, 0x4765C2ADu, 0x6C48916Eu, 0x7553A02Fu, 0x3A1236E8u, 0x230907A9u, 0x0824546Au, 0x113F652B,
	0x96A779E4u, 0x8FBC48A5u, 0xA4911B66u, 0xBD8A2A27u, 0xF2CBBCE0u, 0xEBD08DA1u, 0xC0FDDE62u, 0xD9E6EF23,
	0x14BCE1BDu, 0x0DA7D0FCu, 0x268A833Fu, 0x3F91B27Eu, 0x70D024B9u, 0x69CB15F8u, 0x42E6463Bu, 0x5BFD777A,
	0xDC656BB5u, 0xC57E5AF4u, 0xEE530937u, 0xF7483876u, 0xB809AEB1u, 0xA1129FF0u, 0x8A3FCC33u, 0x9324FD72,
};

constexpr uint32_t CRC_32_TAB2[256] =
{
	0x00000000u, 0x01C26A37u, 0x0384D46Eu, 0x0246BE59u, 0x0709A8DCu, 0x06CBC2EBu, 0x048D7CB2u, 0x054F1685,
	0x0E1351B8u, 0x0FD13B8Fu, 0x0D9785D6u, 0x0C55EFE1u, 0x091AF964u, 0x08D89353u, 0x0A9E2D0Au, 0x0B5C473D,
	0x1C26A370u, 0x1DE4C947u, 0x1FA2771Eu, 0x1E601D29u, 0x1B2F0BACu, 0x1AED619Bu, 0x18ABDFC2u, 0x1969B5F5,
	0x1235F2C8u, 0x13F798FFu, 0x11B126A6u, 0x10734C91u, 0x153C5A14u, 0x14FE3023u, 0x16B88E7Au, 0x177AE44D,
	0x384D46E0u, 0x398F2CD7u, 0x3BC9928Eu, 0x3A0BF8B9u, 0x3F44EE3Cu, 0x3E86840Bu, 0x3CC03A52u, 0x3D025065,
	0x365E1758u, 0x379C7D6Fu, 0x35DAC336u, 0x3418A901u, 0x3157BF84u, 0x3095D5B3u, 0x32D36BEAu, 0x331101DD,
	0x246BE590u, 0x25A98FA7u, 0x27EF31FEu, 0x262D5BC9u, 0x23624D4Cu, 0x22A0277Bu, 0x20E69922u, 0x2124F315,
	0x2A78B428u, 0x2BBADE1Fu, 0x29FC6046u, 0x283E0A71u, 0x2D711CF4u, 0x2CB376C3u, 0x2EF5C89Au, 0x2F37A2AD,
	0x709A8DC0u, 0x7158E7F7u, 0x731E59AEu, 0x72DC3399u, 0x7793251Cu, 0x76514F2Bu, 0x7417F172u, 0x75D59B45,
	0x7E89DC78u, 0x7F4BB64Fu, 0x7D0D0816u, 0x7CCF6221u, 0x798074A4u, 0x78421E93u, 0x7A04A0CAu, 0x7BC6CAFD,
	0x6CBC2EB0u, 0x6D7E4487u, 0x6F38FADEu, 0x6EFA90E9u, 0x6BB5866Cu, 0x6A77EC5Bu, 0x68315202u, 0x69F33835,
	0x62AF7F08u, 0x636D153Fu, 0x612BAB66u, 0x60E9C151u, 0x65A6D7D4u, 0x6464BDE3u, 0x662203BAu, 0x67E0698D,
	0x48D7CB20u, 0x4915A117u, 0x4B531F4Eu, 0x4A917579u, 0x4FDE63FCu, 0x4E1C09CBu, 0x4C5AB792u, 0x4D98DDA5,
	0x46C49A98u, 0x4706F0AFu, 0x45404EF6u, 0x448224C1u, 0x41CD3244u, 0x400F5873u, 0x4249E62Au, 0x438B8C1D,
	0x54F16850u, 0x55330267u, 0x5775BC3Eu, 0x56B7D609u, 0x53F8C08Cu, 0x523AAABBu, 0x507C14E2u, 0x51BE7ED5,
	0x5AE239E8u, 0x5B2053DFu, 0x5966ED86u, 0x58A487B1u, 0x5DEB9134u, 0x5C29FB03u, 0x5E6F455Au, 0x5FAD2F6D,
	0xE1351B80u, 0xE0F771B7u, 0xE2B1CFEEu, 0xE373A5D9u, 0xE63CB35Cu, 0xE7FED96Bu, 0xE5B86732u, 0xE47A0D05,
	0xEF264A38u, 0xEEE4200Fu, 0xECA29E56u, 0xED60F461u, 0xE82FE2E4u, 0xE9ED88D3u, 0xEBAB368Au, 0xEA695CBD,
	0xFD13B8F0u, 0xFCD1D2C7u, 0xFE976C9Eu, 0xFF5506A9u, 0xFA1A102Cu, 0xFBD87A1Bu, 0xF99EC442u, 0xF85CAE75,
	0xF300E948u, 0xF2C2837Fu, 0xF0843D26u, 0xF1465711u, 0xF4094194u, 0xF5CB2BA3u, 0xF78D95FAu, 0xF64FFFCD,
	0xD9785D60u, 0xD8BA3757u, 0xDAFC890Eu, 0xDB3EE339u, 0xDE71F5BCu, 0xDFB39F8Bu, 0xDDF521D2u, 0xDC374BE5,
	0xD76B0CD8u, 0xD6A966EFu, 0xD4EFD8B6u, 0xD52DB281u, 0xD062A404u, 0xD1A0CE33u, 0xD3E6706Au, 0xD2241A5D,
	0xC55EFE10u, 0xC49C9427u, 0xC6DA2A7Eu, 0xC7184049u, 0xC25756CCu, 0xC3953CFBu, 0xC1D382A2u, 0xC011E895,
	0xCB4DAFA8u, 0xCA8FC59Fu, 0xC8C97BC6u, 0xC90B11F1u, 0xCC440774u, 0xCD866D43u, 0xCFC0D31Au, 0xCE02B92D,
	0x91AF9640u, 0x906DFC77u, 0x922B422Eu, 0x93E92819u, 0x96A63E9Cu, 0x976454ABu, 0x9522EAF2u, 0x94E080C5,
	0x9FBCC7F8u, 0x9E7EADCFu, 0x9C381396u, 0x9DFA79A1u, 0x98B56F24u, 0x99770513u, 0x9B31BB4Au, 0x9AF3D17D,
	0x8D893530u, 0x8C4B5F07u, 0x8E0DE15Eu, 0x8FCF8B69u, 0x8A809DECu, 0x8B42F7DBu, 0x89044982u, 0x88C623B5,
	0x839A6488u, 0x82580EBFu, 0x801EB0E6u, 0x81DCDAD1u, 0x8493CC54u, 0x8551A663u, 0x8717183Au, 0x86D5720D,
	0xA9E2D0A0u, 0xA820BA97u, 0xAA6604CEu, 0xABA46EF9u, 0xAEEB787Cu, 0xAF29124Bu, 0xAD6FAC12u, 0xACADC625,
	0xA7F18118u, 0xA633EB2Fu, 0xA4755576u, 0xA5B73F41u, 0xA0F829C4u, 0xA13A43F3u, 0xA37CFDAAu, 0xA2BE979D,
	0xB5C473D0u, 0xB40619E7u, 0xB640A7BEu, 0xB782CD89u, 0xB2CDDB0Cu, 0xB30FB13Bu, 0xB1490F62u, 0xB08B6555,
	0xBBD72268u, 0xBA15485Fu, 0xB853F606u, 0xB9919C31u, 0xBCDE8AB4u, 0xBD1CE083u, 0xBF5A5EDAu, 0xBE9834ED,
};

constexpr uint32_t CRC_32_TAB3[256] =
{
	0x00000000u, 0xB8BC6765u, 0xAA09C88Bu, 0x12B5AFEEu, 0x8F629757u, 0x37DEF032u, 0x256B5FDCu, 0x9DD738B9,
	0xC5B428EFu, 0x7D084F8Au, 0x6FBDE064u, 0xD7018701u, 0x4AD6BFB8u, 0xF26AD8DDu, 0xE0DF7733u, 0x58631056,
	0x5019579Fu, 0xE8A530FAu, 0xFA109F14u, 0x42ACF871u, 0xDF7BC0C8u, 0x67C7A7ADu, 0x75720843u, 0xCDCE6F26,
	0x95AD7F70u, 0x2D111815u, 0x3FA4B7FBu, 0x8718D09Eu, 0x1ACFE827u, 0xA2738F42u, 0xB0C620ACu, 0x087A47C9,
	0xA032AF3Eu, 0x188EC85Bu, 0x0A3B67B5u, 0xB28700D0u, 0x2F503869u, 0x97EC5F0Cu, 0x8559F0E2u, 0x3DE59787,
	0x658687D1u, 0xDD3AE0B4u, 0xCF8F4F5Au, 0x7733283Fu, 0xEAE41086u, 0x525877E3u, 0x40EDD80Du, 0xF851BF68,
	0xF02BF8A1u, 0x48979FC4u, 0x5A22302Au, 0xE29E574Fu, 0x7F496FF6u, 0xC7F50893u, 0xD540A77Du, 0x6DFCC018,
	0x359FD04Eu, 0x8D23B72Bu, 0x9F9618C5u, 0x272A7FA0u, 0xBAFD4719u, 0x0241207Cu, 0x10F48F92u, 0xA848E8F7,
	0x9B14583Du, 0x23A83F58u, 0x311D90B6u, 0x89A1F7D3u, 0x1476CF6Au, 0xACCAA80Fu, 0xBE7F07E1u, 0x06C36084,
	0x5EA070D2u, 0xE61C17B7u, 0xF4A9B859u, 0x4C15DF3Cu, 0xD1C2E785u, 0x697E80E0u, 0x7BCB2F0Eu, 0xC377486B,
	0xCB0D0FA2u, 0x73B168C7u, 0x6104C729u, 0xD9B8A04Cu, 0x446F98F5u, 0xFCD3FF90u, 0xEE66507Eu, 0x56DA371B,
	0x0EB9274Du, 0xB6054028u, 0xA4B0EFC6u, 0x1C0C88A3u, 0x81DBB01Au, 0x3967D77Fu, 0x2BD27891u, 0x936E1FF4,
	0x3B26F703u, 0x839A9066u, 0x912F3F88u, 0x299358EDu, 0xB4446054u, 0x0CF80731u, 0x1E4DA8DFu, 0xA6F1CFBA,
	0xFE92DFECu, 0x462EB889u, 0x549B1767u, 0xEC277002u, 0x71F048BBu, 0xC94C2FDEu, 0xDBF98030u, 0x6345E755,
	0x6B3FA09Cu, 0xD383C7F9u, 0xC1366817u, 0x798A0F72u, 0xE45D37CBu, 0x5CE150AEu, 0x4E54FF40u, 0xF6E89825,
	0xAE8B8873u, 0x1637EF16u, 0x048240F8u, 0xBC3E279Du, 0x21E91F24u, 0x99557841u, 0x8BE0D7AFu, 0x335CB0CA,
	0xED59B63Bu, 0x55E5D15Eu, 0x47507EB0u, 0xFFEC19D5u, 0x623B216Cu, 0xDA874609u, 0xC832E9E7u, 0x708E8E82,
	0x28ED9ED4u, 0x9051F9B1u, 0x82E4565Fu, 0x3A58313Au, 0xA78F0983u, 0x1F336EE6u, 0x0D86C108u, 0xB53AA66D,
	0xBD40E1A4u, 0x05FC86C1u, 0x1749292Fu, 0xAFF54E4Au, 0x322276F3u, 0x8A9E1196u, 0x982BBE78u, 0x2097D91D,
	0x78F4C94Bu, 0xC048AE2Eu, 0xD2FD01C0u, 0x6A4166A5u, 0xF7965E1Cu, 0x4F2A3979u, 0x5D9F9697u, 0xE523F1F2,
	0x4D6B1905u, 0xF5D77E60u, 0xE762D18Eu, 0x5FDEB6EBu, 0xC2098E52u, 0x7AB5E937u, 0x680046D9u, 0xD0BC21BC,
	0x88DF31EAu, 0x3063568Fu, 0x22D6F961u, 0x9A6A9E04u, 0x07BDA6BDu, 0xBF01C1D8u, 0xADB46E36u, 0x15080953,
	0x1D724E9Au, 0xA5CE29FFu, 0xB77B8611u, 0x0FC7E174u, 0x9210D9CDu, 0x2AACBEA8u, 0x38191146u, 0x80A57623,
	0xD8C66675u, 0x607A0110u, 0x72CFAEFEu, 0xCA73C99Bu, 0x57A4F122u, 0xEF189647u, 0xFDAD39A9u, 0x45115ECC,
	0x764DEE06u, 0xCEF18963u, 0xDC44268Du, 0x64F841E8u, 0xF92F7951u, 0x41931E34u, 0x5326B1DAu, 0xEB9AD6BF,
	0xB3F9C6E9u, 0x0B45A18Cu, 0x19F00E62u, 0xA14C6907u, 0x3C9B51BEu, 0x842736DBu, 0x96929935u, 0x2E2EFE50,
	0x2654B999u, 0x9EE8DEFCu, 0x8C5D7112u, 0x34E11677u, 0xA9362ECEu, 0x118A49ABu, 0x033FE645u, 0xBB838120,
	0xE3E09176u, 0x5B5CF613u, 0x49E959FDu, 0xF1553E98u, 0x6C820621u, 0xD43E6144u, 0xC68BCEAAu, 0x7E37A9CF,
	0xD67F4138u, 0x6EC3265Du, 0x7C7689B3u, 0xC4CAEED6u, 0x591DD66Fu, 0xE1A1B10Au, 0xF3141EE4u, 0x4BA87981,
	0x13CB69D7u, 0xAB770EB2u, 0xB9C2A15Cu, 0x017EC639u, 0x9CA9FE80u, 0x241599E5u, 0x36A0360Bu, 0x8E1C516E,
	0x866616A7u, 0x3EDA71C2u, 0x2C6FDE2Cu, 0x94D3B949u, 0x090481F0u, 0xB1B8E695u, 0xA30D497Bu, 0x1BB12E1E,
	0x43D23E48u, 0xFB6E592Du, 0xE9DBF6C3u, 0x516791A6u, 0xCCB0A91Fu, 0x740CCE7Au, 0x66B96194u, 0xDE0506F1,
};

#endif

CRC32::CRC32() noexcept
{
	Reset();
}

void CRC32::Update(char c) noexcept
{
	crc = (CRC_32_TAB[(crc ^ (uint8_t)c) & 0xFF] ^ (crc >> 8));
}

// A note on CRC algorithms on ARM:
// Original algorithm (1 byte per loop iteration, 1K table): 7 instructions, 11 clocks (11 clocks/byte)
// Algorithm currently used on non-SAME70/SAME5x processors (4 bytes per loop iteration, 1K table): 19 instructions, 26 clocks (6.5 clocks/byte)
// Slicing-by-4 using 1 dword per loop iteration: 15 instructions, 18 clocks (4.5 clocks/byte)
// Slicing-by-4 using 1 quadword per loop iteration: 28 instructions, 31 clocks (3.875 clocks/byte)
void CRC32::Update(const char *_ecv_array s, size_t len) noexcept
{
	// The speed of this function affects the speed of file uploads, so make it as fast as possible. Sadly the SAME70 doesn't do hardware CRC calculation.
	const char *_ecv_array const end = s + len;

#if USE_SAME5x_HARDWARE_CRC
	if (len >= 26)								// 26 is about the optimum changeover point
	{
		uint32_t reflectedCrc = Reflect(crc);
		TaskCriticalSectionLocker lock;			// we need exclusive use of the CRC unit

		if ((reinterpret_cast<uint32_t>(s) & 3) != 0)
		{
			// Process any bytes at the start until we reach a dword boundary
			DMAC->CRCCTRL.reg = DMAC_CRCCTRL_CRCBEATSIZE_BYTE | DMAC_CRCCTRL_CRCSRC_DISABLE | DMAC_CRCCTRL_CRCPOLY_CRC32;	// disable the CRC unit
			DMAC->CRCCHKSUM.reg = reflectedCrc;
			DMAC->CRCCTRL.reg = DMAC_CRCCTRL_CRCBEATSIZE_BYTE | DMAC_CRCCTRL_CRCSRC_IO | DMAC_CRCCTRL_CRCPOLY_CRC32;
			do
			{
				DMAC->CRCDATAIN.reg = *s++;
			} while ((reinterpret_cast<uint32_t>(s) & 3) != 0 && s != end);

			reflectedCrc = DMAC->CRCCHKSUM.reg;
			DMAC->CRCSTATUS.reg = DMAC_CRCSTATUS_CRCBUSY;
		}

		// Process a whole number of dwords
		const char * const endAligned = s + ((end - s) & ~3);
		if (s != endAligned)
		{
			DMAC->CRCCTRL.reg = DMAC_CRCCTRL_CRCBEATSIZE_WORD | DMAC_CRCCTRL_CRCSRC_DISABLE | DMAC_CRCCTRL_CRCPOLY_CRC32;	// disable the CRC unit
			DMAC->CRCCHKSUM.reg = reflectedCrc;
			DMAC->CRCCTRL.reg = DMAC_CRCCTRL_CRCBEATSIZE_WORD | DMAC_CRCCTRL_CRCSRC_IO | DMAC_CRCCTRL_CRCPOLY_CRC32;
			do
			{
				DMAC->CRCDATAIN.reg = *reinterpret_cast<const uint32_t*>(s);
				s += 4;
			} while (s != endAligned);

			reflectedCrc = DMAC->CRCCHKSUM.reg;
			DMAC->CRCSTATUS.reg = DMAC_CRCSTATUS_CRCBUSY;
		}

		// Process up to 3 bytes at the end
		if (s != end)
		{
			DMAC->CRCCTRL.reg = DMAC_CRCCTRL_CRCBEATSIZE_BYTE | DMAC_CRCCTRL_CRCSRC_DISABLE | DMAC_CRCCTRL_CRCPOLY_CRC32;	// disable the CRC unit
			DMAC->CRCCHKSUM.reg = reflectedCrc;
			DMAC->CRCCTRL.reg = DMAC_CRCCTRL_CRCBEATSIZE_BYTE | DMAC_CRCCTRL_CRCSRC_IO | DMAC_CRCCTRL_CRCPOLY_CRC32;
			do
			{
				DMAC->CRCDATAIN.reg = *s++;
			}
			while (s != end);

			reflectedCrc = DMAC->CRCCHKSUM.reg;
			DMAC->CRCSTATUS.reg = DMAC_CRCSTATUS_CRCBUSY;
		}
		crc = Reflect(reflectedCrc);
	}
	else
#endif
	{
		// Work on a local copy of the crc to avoid storing it all the time
		uint32_t locCrc = crc;

		// Process any bytes at the start until we reach a dword boundary
		while ((reinterpret_cast<uint32_t>(s) & 3) != 0 && s != end)
		{
			locCrc = (CRC_32_TAB[(locCrc ^ (uint8_t)*s++) & 0xFF] ^ (locCrc >> 8));
		}

# if SAME70 || (SAME5x && !USE_SAME5x_HARDWARE_CRC)
		// Process an whole number of quadwords
		const char * const endAligned = s + ((end - s) & ~7);
		while (s != endAligned)
		{
			// Slicing-by-4 algorithm, 2 dwords at a time
			const uint32_t data0 = *reinterpret_cast<const uint32_t*>(s) ^ locCrc;
			locCrc = CRC_32_TAB[(data0 >> 24) & 0xFF] ^ CRC_32_TAB1[(data0 >> 16) & 0xFF] ^ CRC_32_TAB2[(data0 >> 8) & 0xFF] ^ CRC_32_TAB3[data0 & 0xFF];
			const uint32_t data1 = *reinterpret_cast<const uint32_t*>(s + 4) ^ locCrc;
			locCrc = CRC_32_TAB[(data1 >> 24) & 0xFF] ^ CRC_32_TAB1[(data1 >> 16) & 0xFF] ^ CRC_32_TAB2[(data1 >> 8) & 0xFF] ^ CRC_32_TAB3[data1 & 0xFF];
			s += 8;
		}
# else
		// Process a whole number of dwords
		const char *_ecv_array const endAligned = s + ((size_t)(end - s) & (size_t)~3);
		while (s != endAligned)
		{
			const uint32_t data = *reinterpret_cast<const uint32_t*>(s);
			s += 4;
			locCrc = (CRC_32_TAB[(locCrc ^ data) & 0xFF] ^ (locCrc >> 8));
			locCrc = (CRC_32_TAB[(locCrc ^ (data >> 8)) & 0xFF] ^ (locCrc >> 8));
			locCrc = (CRC_32_TAB[(locCrc ^ (data >> 16)) & 0xFF] ^ (locCrc >> 8));
			locCrc = (CRC_32_TAB[(locCrc ^ (data >> 24)) & 0xFF] ^ (locCrc >> 8));
		}
# endif

		// Process up to 7 (SAME70/SAME5x) or 3 (others) bytes at the end
		while (s != end)
		{
			locCrc = (CRC_32_TAB[(locCrc ^ (uint8_t)*s++) & 0xFF] ^ (locCrc >> 8));
		}

		crc = locCrc;
	}
}

#if USE_SAME5x_HARDWARE_CRC

// Special function used to CRC a whole number of 32-bit words aligned on a word boundary, used to check for memory corruption
uint32_t CRC32::CalcCRC32(const uint32_t *_ecv_array data, const uint32_t *_ecv_array end) noexcept
{
	DMAC->CRCCTRL.reg = DMAC_CRCCTRL_CRCBEATSIZE_WORD | DMAC_CRCCTRL_CRCSRC_DISABLE | DMAC_CRCCTRL_CRCPOLY_CRC32;	// disable the CRC unit
	DMAC->CRCCHKSUM.reg = 0xFFFFFFFF;
	DMAC->CRCCTRL.reg = DMAC_CRCCTRL_CRCBEATSIZE_WORD | DMAC_CRCCTRL_CRCSRC_IO | DMAC_CRCCTRL_CRCPOLY_CRC32;
	while (data < end)
	{
		DMAC->CRCDATAIN.reg = *data++;
	}
	const uint32_t rslt = DMAC->CRCCHKSUM.reg;
	DMAC->CRCSTATUS.reg = DMAC_CRCSTATUS_CRCBUSY;
	return rslt;
}

#endif

// End
