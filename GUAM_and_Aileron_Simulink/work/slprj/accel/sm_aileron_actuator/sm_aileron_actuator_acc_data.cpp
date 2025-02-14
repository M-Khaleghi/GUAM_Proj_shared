#include "sm_aileron_actuator_acc.h"
#include "sm_aileron_actuator_acc_private.h"
const ConstB_sm_aileron_actuator_T sm_aileron_actuator_rtInvariant = { 0.0 ,
0.0 , 0.0 , 0.0 , 0.0 , 0.0 } ; P_sm_aileron_actuator_T
sm_aileron_actuator_rtDefaultP = { { 1.0 , 1.0 } , 0.005 , { 1.0 , 1.0 } ,
0.0001 , { 1.0 , 1.0 } , 0.35 , { 1.0 , 1.0 } , 0.09 , { 1.0 , 1.0 } , 0.005
, { 1.0 , 1.0 } , 0.0001 , { 1.0 , 1.0 } , 0.35 , { 1.0 , 1.0 } , 0.09 , {
1.0 , 1.0 } , 0.005 , { 1.0 , 1.0 } , 0.0001 , { 1.0 , 1.0 } , 0.35 , { 1.0 ,
1.0 } , 0.09 , { 1.0 , 1.0 } , 0.35 , { 1.0 , 1.0 } , 0.09 , { 1.0 , 1.0 } ,
0.35 , { 1.0 , 1.0 } , 0.09 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 ,
57.295779513082316 , { - 0.99990920426259511 , - 0.999329299739067 , -
0.99505475368673046 , - 0.9640275800758169 , - 0.76159415595576485 , 0.0 ,
0.76159415595576485 , 0.9640275800758169 , 0.99505475368673046 ,
0.999329299739067 , 0.99990920426259511 } , { - 5.0 , - 4.0 , - 3.0 , - 2.0 ,
- 1.0 , 0.0 , 1.0 , 2.0 , 3.0 , 4.0 , 5.0 } , { - 0.99990920426259511 , -
0.999329299739067 , - 0.99505475368673046 , - 0.9640275800758169 , -
0.76159415595576485 , 0.0 , 0.76159415595576485 , 0.9640275800758169 ,
0.99505475368673046 , 0.999329299739067 , 0.99990920426259511 } , { - 5.0 , -
4.0 , - 3.0 , - 2.0 , - 1.0 , 0.0 , 1.0 , 2.0 , 3.0 , 4.0 , 5.0 } , { -
0.99990920426259511 , - 0.999329299739067 , - 0.99505475368673046 , -
0.9640275800758169 , - 0.76159415595576485 , 0.0 , 0.76159415595576485 ,
0.9640275800758169 , 0.99505475368673046 , 0.999329299739067 ,
0.99990920426259511 } , { - 5.0 , - 4.0 , - 3.0 , - 2.0 , - 1.0 , 0.0 , 1.0 ,
2.0 , 3.0 , 4.0 , 5.0 } , 0.5 , 1.0E-6 , { 0.0 , 0.0 , 0.0 , 0.0 , 5.0 , 10.0
, 10.0 , 0.0 , 0.0 , - 10.0 , - 10.0 , - 10.0 , 0.0 , 0.0 , 0.0 , 6.0 , 15.0
, 15.0 , 6.0 , 0.0 , - 5.0 , - 5.0 , 0.0 , 0.0 , 5.0 , 10.0 , 10.0 } , { 0.0
, 0.5 , 1.0 , 1.5 , 2.0 , 2.5 , 3.0 , 3.5 , 4.0 , 4.5 , 5.0 , 5.5 , 6.0 , 6.5
, 7.0 , 7.5 , 8.0 , 8.5 , 9.0 , 9.5 , 10.0 , 10.5 , 11.0 , 11.5 , 12.0 , 12.5
, 13.0 } , 0.0 , 5.0 , - 5.0 , - 125.66370614359172 , 125.66370614359172 ,
0.0 , 5.0 , - 5.0 , - 125.66370614359172 , 125.66370614359172 , 0.3 , 5.0 ,
5.0 , - 5.0 , 10.0 , 5.0 , 5.0 , 0.0 , 0.0 , 5.0 , - 5.0 , -
125.66370614359172 , 125.66370614359172 , 0.0 , 5.0 , - 5.0 , -
125.66370614359172 , 125.66370614359172 , 0.3 , 5.0 , 5.0 , - 5.0 , 10.0 ,
5.0 , 5.0 , 0.0 , 0.0 , 5.0 , - 5.0 , - 125.66370614359172 ,
125.66370614359172 , 0.0 , 5.0 , - 5.0 , - 125.66370614359172 ,
125.66370614359172 , 0.3 , 5.0 , 5.0 , - 5.0 , 10.0 , 5.0 , 5.0 , 0.0 , 100.0
, 0.0 , 0.05 , 1.0 , 0.01 , 0.0 , 0.0 , 0.0 , 0.0 , 0.05 , 1.0 , 0.01 , 0.0 ,
0.0 , 0.0 , 0.0 , { 0.0 , 0.0 , 0.0 , 0.0 , 1.0 } , 0.0 , { - 0.0 , - 0.0 , -
0.0 , - 0.0 , - 0.0 , - 0.0 , - 0.0 , - 0.0 , - 0.0 } , { 2.0 , 2.0 , 2.0 ,
2.0 , 2.0 , 2.0 , 2.0 , 2.0 , 2.0 } , - 1.0 , - 2.0 , 2.0 , 1.0 , { 0.5 , 0.5
, 0.5 , 0.5 , 0.5 } , { 0.0 , 0.0 , 0.0 , 0.0 , 0.0 } , 0.0 , 0.1 , 0.0 , 0.0
, 0.0 , 0.1 , 0.0 , 0.1 , 0.0 , 100.0 , 0.0 , 100.0 , 0.0 , 0.05 , 1.0 , 0.01
, 0.0 , 0.0 , 0.0 , 0.0 , 0.05 , 1.0 , 0.01 , 0.0 , 0.0 , 0.0 , 0.0 , 100.0 ,
0.0 , 0.05 , 1.0 , 0.01 , 0.0 , 0.0 , 0.0 , 0.0 , 0.05 , 1.0 , 0.01 , 0.0 ,
0.0 , 0.0 , 0.0 , 100.0 , 0.0 , 0.0 , 3.0 , 0.3 , 1.0 , 3.0 , 0.3 , 1.0 , 3.0
, 0.3 , 1.0 , 0.1 , 0.0 , { - 0.11290214720089338 , 0.29593032439892497 ,
0.15618386150377961 , 0.43440495586154021 , 0.30872373495492633 ,
0.036515234051006561 , 0.87916594091038525 , - 0.42833197233026954 ,
0.54386036467085808 } , { - 3.8116734075170169 , - 0.010910285027805316 ,
4.0727064860709126 , - 1.4204152766010465 , - 0.51515780062009719 ,
8.1486045326562362 , - 0.086424032419304742 , - 2.2515984457815046 , -
2.4253202531761149 } , { 0.62273316308553384 , - 1.7818702727081337 , -
1.7671080334100375 , 3.66389873452404 , 4.0996179747587975 , -
9.4968753376982384 , 3.2020350782589682 , 0.13838737342777774 ,
3.561052844522798 } , { 0.74143101943693734 , - 0.02734104541333128 ,
0.47119685422746643 , 0.064159532165514135 , 0.41357657538640441 , -
0.7703406271690092 , 0.42147115522944656 , 0.74877778622314584 ,
0.59421333564936452 } , { 0.62414193480475333 , 0.255530780606044 ,
1.1790232217846763 , - 0.86942413381675043 , - 0.7635384939093397 , -
0.81516518053274223 , - 1.9647023561242183 , 0.81138605008618 ,
0.477053848731245 } , { - 5.3551332091200523 , 1.9475610307918536 ,
2.5929883529776268 , 0.68456159149053908 , 0.325795938670692 , -
2.1592640449572293 , 0.96936436384287172 , 1.1271148737315408 ,
1.0889322217647679 } , { - 1.0305705925332935 , 0.70726302158887533 ,
0.41342164180942187 , - 0.953330860960186 , - 0.41722168502132456 , -
1.9042444170917492 , 1.3034439118025274 , - 0.068257525670697816 , -
0.41203473824036241 } , { 1.2020850181943634 , - 0.23123426080004916 , -
0.16495992727348976 , 1.5352964499869819 , 4.5947443640461039 , -
1.1220767467100123 , 2.0580358390360325 , - 0.048673963349619986 , -
0.064668510196518855 } , { - 0.64564456543929716 , - 0.43296321690547956 , -
1.7426913876415779 , - 0.53684544812415513 , - 0.776532656609463 , -
2.3720128562048712 , - 0.068007191164881034 , 0.0041299960746167307 , -
0.17506097792393377 } , { 7.01773769071809 , 1.7201811049594156 ,
2.9164401831993336 , 2.5988921647650725 , - 6.3763635276528285 ,
3.9333218058105235 , - 1.7347949398367206 , - 0.73590359314481835 , -
0.62621121272786706 } , { - 1.3001532850699777 , 0.64030090823591534 , -
0.20732074934784181 , 3.2136060930707853 , - 0.11162504203938921 ,
0.36045493510463156 , - 0.13548705192745419 , - 1.2107106304748718 , -
0.50656347802919588 } , { - 0.42556867795499337 , 1.8416562909451251 , -
1.228470245011468 , 0.37244694269390372 , 0.95698189229153052 ,
0.37440047490378747 , 2.881037164159459 , - 1.7506313526972537 , -
0.70850063801947039 } , { - 3.4732787436890304 , 0.93698493080013034 ,
0.70249374335857162 , 3.4477186489001732 , 0.9989709960084876 , -
0.21587278622403647 , 3.6297102265296828 , - 0.52513130217490844 , -
4.3267658042462021 } , { - 0.6506852113808117 , 0.15420784534762241 , -
0.1782130481575957 , 0.49776590150949973 , 1.4340298113819798 , -
1.4574274982045288 , 1.6275370728095768 , 1.1680423016108874 ,
1.0164937158756571 } , { 0.68941212249675932 , 0.64023425899787889 , -
3.6879524262878025 , 0.47600887675280762 , 0.67979929852279353 , -
0.74609015589304573 , - 0.35768816925711833 , 0.48161852843818964 ,
0.55953940480860509 } , { - 1.1545363156440172 , 1.4330448171951577 , -
1.3395376156024428 , 0.24132522002965562 , 2.2710670835856259 ,
0.12651790117180828 , 4.24000612104515 , - 1.3156091257984261 , -
1.1486609263510841 } , { 2.3749690755837571 , - 0.5458215602249602 ,
2.0803423555354845 , - 5.6225347024412331 , - 1.0378152026621144 ,
4.765442503327332 , - 2.6197534064932446 , 0.18276304426540457 , -
0.815026966656692 } , { - 0.19456969115400788 , - 0.0033975889631352842 ,
0.59461706737466158 , 0.79251072156966829 , - 0.59660655975958887 ,
2.1557197446933483 , - 1.3609387133428796 , - 0.52695620974853918 , -
1.046475247756 } , { - 2.8028130372529407 , 1.9846833109787196 ,
4.4110849099989924 , 2.4313326667453929 , 8.6985339720986357 , -
3.5840896158723425 , 2.5431052807127448 , 2.7218717898544735 ,
2.7099496368976164 } , { - 4.71034955106154 , - 5.8783706010213423 ,
3.42660407844798 , 6.345399028323377 , 7.2409976524603854 , -
5.124535452957363 , 3.3135449354284923 , - 4.7624796832496346 ,
7.5067668541303787 } , { 1.1286495408352424 , - 3.314041527644469 ,
0.58620586478322234 , - 1.9612328887258359 , - 3.2101006957343645 ,
0.74163720979313641 , 0.76379372408596136 , 4.8299185258627082 ,
2.1546276657482952 , - 2.5385341845743459 , 2.2277508771590671 ,
0.10484703396832451 , 1.6124543968907514 , 1.9166087540456569 , -
1.4538501188947985 , 1.2582226307610651 , - 2.0046877567331065 ,
5.3119189271275609 , 4.23006550015287 , - 7.6350949995587367 } , 1.0 , 1.0 ,
{ 0.048916811232522568 , 0.013758803416667759 , 0.057472946203686634 ,
0.018354079103602892 , - 0.028640482147416942 , 0.057086645075960361 ,
0.10717314887063321 , 1.0168481867431465 , 0.010834779866849843 , -
0.018576858002613363 , - 0.0061061327109310495 , - 0.024609069716369746 , -
0.03322650018086247 , - 1.061621372097568 , 0.025681485004616946 , -
0.043489444036858574 , 0.074416827029532728 , - 1.0292894118971296 , -
0.013342261743951171 , 0.041162698126067536 } , { - 0.38229903968385 , -
0.5117266292235999 , - 0.78508056109989244 , 1.19194014340228 ,
0.00818145756903862 , - 0.45578585606785754 , - 0.63065329132687975 ,
0.49325501843665304 , 0.035707304399465595 , 1.0453418841100142 , -
0.33997542109612611 , 0.65012683342947275 , - 0.9244131989188088 , -
0.26951790430276656 , 0.774893629904248 , - 0.4216688407983053 , -
1.3824582058154151 , - 0.51611403265433975 , - 0.056434179877843681 , -
0.30704995673136309 } , { - 1.2880790497551351 , - 0.72905029884420236 ,
0.30827258839257787 , 0.36117470811109009 , 0.48418626812845744 , -
0.91773687891439737 , - 1.5248637344227234 , - 0.42966776255878242 ,
0.29593129011607266 , 1.2326419332762746 , - 0.49054474033843293 , -
0.95363328290838234 , - 0.14590349778556114 , - 0.97001718691537619 ,
1.1995032537719852 , 0.14977085506167381 , - 1.5975374475051156 ,
0.50221346905230635 , 0.32042924207559015 , - 0.71446247574571087 } , { -
0.031012359331137714 , - 1.2613702623503418 , 0.91176533305249075 , -
0.097584708927178659 , 1.679683068307424 , 0.10782797345277419 ,
0.95344242351078057 , - 1.2622410018652164 , - 0.77182323872987046 , -
0.43668150915729725 , 0.20353531942711209 , - 0.62734334606974207 ,
1.0224612858049835 , 1.970784840679044 , - 0.86288314302170432 ,
0.97830730988033265 , - 0.026742745287518754 , 1.3723039911783979 , -
0.60637699883366225 , - 0.97034373983368272 } , { 1.1197090724880892 ,
1.8400160618188868 , - 2.7730982122197929 , 1.9111798644335887 , -
2.90376596439842 , - 0.18151383161655907 , - 0.78900028666384325 ,
1.863804790537068 , 0.2028827082403252 , 1.1656948974129282 ,
0.26005059202264574 , 4.2084550331597956 , - 2.74303823537212 ,
0.26393653104309306 , 1.0851682126257123 , - 1.8006525261733062 , -
0.51098310414952919 , - 2.2161423968541447 , - 0.47923080203010138 ,
0.71320674727922906 } , { 0.012043356667263569 , 0.75786212734135028 ,
0.44593369379244929 , - 1.6191039752398455 , 2.7433661531224285 } , 0.0 ,
30.0 , 0.0 , { 0.0 , 0.0 , 1.0 , 1.0 , 0.0 } , { 0.0 , 1.0 , 1.0 , 0.0 , 0.0
} , - 1.0 , { 0.0 , 1.0 , 1.0 , 0.0 , 1.0 } , - 50.0 , { 0.0 , 1.0 , 0.0 ,
0.0 , 0.0 } , { 0.0 , 0.0 , 1.0 , 1.0 , 1.0 } , - 30.0 , { 0.0 , 1.0 , 0.0 ,
0.0 , 1.0 } , 9.8 , { 0.0 , 1.0 , 0.0 , 1.0 , 1.0 } , - 0.007 , { 0.0 , 1.0 ,
0.0 , 1.0 , 0.0 } , 0.0 , 2.0 , { 0.0 , 0.0 , 0.0 , 0.0 , 0.0 } , 0.0 , 0.0 ,
6.1 , { 0.0 , 0.0 , 0.0 , 0.0 , 0.0 } , 0.0 , 0.0 , 6.0 , { 0.0 , 0.0 , 0.0 ,
0.0 , 0.0 } , 0.0 , 1.0 , 2.0 , 1.0 , 2.0 , 1.0 , 2.0 , 1.0 , 2.0 , 1.0 , 2.0
, 1.0 , 2.0 , 13.0 , false , false , { 'a' , 'a' , 'a' , 'a' , 'a' , 'a' } }
;
