#include "OpDebugSkiaTests.h"

static void issue12556(skiatest::Reporter* reporter, const char* filename) {
SkPath patha;
      patha.moveTo(SkBits2Float(0x44f61000), SkBits2Float(0x45ce1700));  // 1968.5f, 6594.88f
      patha.lineTo(SkBits2Float(0x45c4d9ab), SkBits2Float(0x45ce1700));  // 6299.21f, 6594.88f
      patha.lineTo(SkBits2Float(0x45c49700), SkBits2Float(0x45ce59ab));  // 6290.88f, 6603.21f
      patha.lineTo(SkBits2Float(0x44f71aab), SkBits2Float(0x45ce59ab));  // 1976.83f, 6603.21f
      patha.lineTo(SkBits2Float(0x44f71aba), SkBits2Float(0x45e061e5));  // 1976.84f, 7180.24f
      patha.lineTo(SkBits2Float(0x45c4970f), SkBits2Float(0x45e061e5));  // 6290.88f, 7180.24f
      patha.lineTo(SkBits2Float(0x45c4d9be), SkBits2Float(0x45e0a493));  // 6299.22f, 7188.57f
      patha.lineTo(SkBits2Float(0x44f61000), SkBits2Float(0x45e0a493));  // 1968.5f, 7188.57f
      patha.lineTo(SkBits2Float(0x44f61000), SkBits2Float(0x45ce1700));  // 1968.5f, 6594.88f
      patha.close();
SkPath pathb;
      pathb.moveTo(SkBits2Float(0x45c4d9be), SkBits2Float(0x45ce1700));  // 6299.22f, 6594.88f
      pathb.lineTo(SkBits2Float(0x45c4d9be), SkBits2Float(0x45e0a493));  // 6299.22f, 7188.57f
      pathb.lineTo(SkBits2Float(0x45c4970f), SkBits2Float(0x45e061e5));  // 6290.88f, 7180.24f
      pathb.lineTo(SkBits2Float(0x45c4970f), SkBits2Float(0x45ce59af));  // 6290.88f, 6603.21f
      pathb.lineTo(SkBits2Float(0x45c4d9be), SkBits2Float(0x45ce1700));  // 6299.22f, 6594.88f
      pathb.close();
SkPathOp op = kUnion_SkPathOp;
    testPathOp(reporter, patha, pathb, op, filename);
}

static void pentrek1(skiatest::Reporter* reporter, const char* filename) {
SkPath b;
b.moveTo({441.505,264.541});
b.quadTo({465.507,252.904}, {486.118,246.702});
b.quadTo({506.339,240.617}, {538.643,233.581});
b.quadTo({570.923,226.55}, {605.627,221.6});
b.quadTo({640.469,216.631}, {668.496,215.479});
b.quadTo({696.491,214.329}, {713.175,215.219});
b.quadTo({730.026,216.119}, {740.911,218.108});
b.cubicTo({740.911,218.108}, {746.341,224.314}, {749.943,224.314});
b.cubicTo({748.95,229.743}, {747.958,238.775}, {742.744,238.775});
b.quadTo({727.685,236.022}, {712.11,235.191});
b.quadTo({696.368,234.351}, {669.317,235.462});
b.quadTo({642.296,236.572}, {608.451,241.4});
b.quadTo({574.468,246.247}, {542.9,253.122});
b.quadTo({511.356,259.993}, {491.882,265.853});
b.quadTo({472.797,271.596}, {450.23,282.537});
b.cubicTo({450.23,282.537}, {445.263,282.868}, {439.277,282.868});
b.cubicTo({436.869,277.902}, {434.461,266.949}, {436.538,266.949});
b.close();
SkPath left(b);
b.reset();
b.moveTo({457.062,207.276});
b.quadTo({459.401,218.797}, {459.756,227.626});
b.quadTo({460.073,235.527}, {460.799,246.604});
b.quadTo({461.449,256.519}, {466.945,272.989});
b.quadTo({472.598,289.93}, {483.772,311.371});
b.quadTo({495.058,333.025}, {505.284,348.272});
b.quadTo({515.746,363.871}, {521.259,371.657});
b.quadTo({525.983,378.328}, {533.581,384.209});
b.cubicTo({533.581,384.209}, {537.946,393.873}, {538.747,393.873});
b.cubicTo({535.369,398.238}, {531.991,403.403}, {525.705,403.403});
b.quadTo({511.384,392.32}, {504.936,383.214});
b.quadTo({499.277,375.222}, {488.673,359.412});
b.quadTo({477.833,343.248}, {466.036,320.614});
b.quadTo({454.129,297.766}, {447.973,279.32});
b.quadTo({441.66,260.402}, {440.842,247.912});
b.quadTo({440.099,236.582}, {439.772,228.428});
b.quadTo({439.482,221.203}, {437.462,211.256});
b.cubicTo({437.462,211.256}, {436.363,200.564}, {439.863,200.564});
b.cubicTo({445.272,199.466}, {450.681,201.867}, {455.963,201.867});
b.close();
    testPathOp(reporter, left, b, kIntersect_SkPathOp, filename);
}


static void pentrek2(skiatest::Reporter* reporter, const char* filename) {
SkPath b;
b.moveTo({0x1.b98132p+8,0x1.088a72p+8});
b.quadTo({0x1.d181dcp+8,0x1.f9cedp+7}, {0x1.e61e5p+8,0x1.ed672ep+7});
b.quadTo({0x1.fa56c8p+8,0x1.e13bb8p+7}, {0x1.0d525ap+9,0x1.d3294ap+7});
b.quadTo({0x1.1d7622p+9,0x1.c5199ep+7}, {0x1.2ed044p+9,0x1.bb334cp+7});
b.quadTo({0x1.403c1p+9,0x1.b142e6p+7}, {0x1.4e3f74p+9,0x1.aef55p+7});
b.quadTo({0x1.5c3edap+9,0x1.aca864p+7}, {0x1.649676p+9,0x1.ae7046p+7});
b.quadTo({0x1.6d034ap+9,0x1.b03cbp+7}, {0x1.7274a8p+9,0x1.b43772p+7});
b.cubicTo({0x1.7274a8p+9,0x1.b43772p+7}, {0x1.752b98p+9,0x1.c0a0dep+7}, {0x1.76f8aap+9,0x1.c0a0dep+7});
b.cubicTo({0x1.7679a4p+9,0x1.cb7c9ep+7}, {0x1.75fa9cp+9,0x1.dd8ca8p+7}, {0x1.735f48p+9,0x1.dd8ca8p+7});
b.quadTo({0x1.6bd7b6p+9,0x1.d80b5p+7}, {0x1.640e0ap+9,0x1.d661bap+7});
b.quadTo({0x1.5c2f26p+9,0x1.d4b39cp+7}, {0x1.4ea88cp+9,0x1.d6ecbp+7});
b.quadTo({0x1.4125fp+9,0x1.d9251ap+7}, {0x1.3039bcp+9,0x1.e2ccb4p+7});
b.quadTo({0x1.1f3bdep+9,0x1.ec7e62p+7}, {0x1.0f7326p+9,0x1.fa3eb6p+7});
b.quadTo({0x1.ff5b38p+8,0x1.03fe24p+8}, {0x1.ebe1bp+8,0x1.09da6ap+8});
b.quadTo({0x1.d8cc24p+8,0x1.0f9898p+8}, {0x1.c23acep+8,0x1.1a898ep+8});
b.cubicTo({0x1.c23acep+8,0x1.1a898ep+8}, {0x1.bd437p+8,0x1.1ade2cp+8}, {0x1.b746d4p+8,0x1.1ade2cp+8});
b.cubicTo({0x1.b4de72p+8,0x1.15e6cep+8}, {0x1.b27612p+8,0x1.0af2d4p+8}, {0x1.b489d4p+8,0x1.0af2d4p+8});
b.close();
SkPath left(b);
b.reset();
b.moveTo({0x1.c90fcep+8,0x1.9e8d24p+7});
b.quadTo({0x1.cb66b2p+8,0x1.b597fp+7}, {0x1.cbc17p+8,0x1.c7409ep+7});
b.quadTo({0x1.cc12a4p+8,0x1.d70ddp+7}, {0x1.cccc84p+8,0x1.ed351ap+7});
b.quadTo({0x1.cd72eap+8,0x1.0084f6p+8}, {0x1.d2f1dcp+8,0x1.10fd28p+8});
b.quadTo({0x1.d8991p+8,0x1.21edfap+8}, {0x1.e3c5bp+8,0x1.375edap+8});
b.quadTo({0x1.ef0ed8p+8,0x1.4d067ap+8}, {0x1.f94898p+8,0x1.5c458ep+8});
b.quadTo({0x1.01df82p+9,0x1.6bdf18p+8}, {0x1.04a12p+9,0x1.73a81cp+8});
b.quadTo({0x1.06fdd6p+9,0x1.7a5418p+8}, {0x1.0aca68p+9,0x1.80357ap+8});
b.cubicTo({0x1.0aca68p+9,0x1.80357ap+8}, {0x1.0cf918p+9,0x1.89df74p+8}, {0x1.0d5fa2p+9,0x1.89df74p+8});
b.cubicTo({0x1.0baf42p+9,0x1.8e3cd2p+8}, {0x1.09fee2p+9,0x1.936746p+8}, {0x1.06da46p+9,0x1.936746p+8});
b.quadTo({0x1.ff6252p+8,0x1.8851e8p+8}, {0x1.f8efbep+8,0x1.7f36e4p+8});
b.quadTo({0x1.f346fcp+8,0x1.7738e8p+8}, {0x1.e8ac68p+8,0x1.676972p+8});
b.quadTo({0x1.ddd528p+8,0x1.573f86p+8}, {0x1.d2095p+8,0x1.409d26p+8});
b.quadTo({0x1.c620fp+8,0x1.29c406p+8}, {0x1.bff924p+8,0x1.1751d8p+8});
b.quadTo({0x1.b9a916p+8,0x1.04670ap+8}, {0x1.b8d77cp+8,0x1.efd2e6p+7});
b.quadTo({0x1.b8195cp+8,0x1.d92a3p+7}, {0x1.b7c59p+8,0x1.c8db62p+7});
b.quadTo({0x1.b77b4ep+8,0x1.ba681p+7}, {0x1.b57632p+8,0x1.a682dcp+7});
b.cubicTo({0x1.b57632p+8,0x1.a682dcp+7}, {0x1.b45d0ap+8,0x1.9120b8p+7}, {0x1.b7dceep+8,0x1.9120b8p+7});
b.cubicTo({0x1.bd4592p+8,0x1.8eee66p+7}, {0x1.c2ae38p+8,0x1.93bbdap+7}, {0x1.c7f6a4p+8,0x1.93bbdap+7});
b.close();
    testPathOp(reporter, left, b, kIntersect_SkPathOp, filename);
}

static void pentrek3(skiatest::Reporter* reporter, const char* filename) {
SkPath b;
b.moveTo({0x1.2cb41ep+9,0x1.4bab78p+8});
b.quadTo({0x1.27abb6p+9,0x1.51fef6p+8}, {0x1.255c2p+9,0x1.55dce8p+8});
b.quadTo({0x1.230d52p+9,0x1.59b98ep+8}, {0x1.205328p+9,0x1.5fbc52p+8});
b.quadTo({0x1.1db57ep+9,0x1.658048p+8}, {0x1.1b7252p+9,0x1.6d2768p+8});
b.quadTo({0x1.195e7ap+9,0x1.742e7p+8}, {0x1.186614p+9,0x1.7e5ab6p+8});
b.quadTo({0x1.176b5p+9,0x1.889fbap+8}, {0x1.186e1cp+9,0x1.973346p+8});
b.quadTo({0x1.1974ccp+9,0x1.a5fee6p+8}, {0x1.1d4782p+9,0x1.b712e8p+8});
b.quadTo({0x1.2127b2p+9,0x1.c86326p+8}, {0x1.27101ep+9,0x1.d7d258p+8});
b.quadTo({0x1.2cfc84p+9,0x1.e74bf2p+8}, {0x1.330cc8p+9,0x1.f168fp+8});
b.quadTo({0x1.3911eep+9,0x1.fb735cp+8}, {0x1.3e2b4p+9,0x1.0030eap+9});
b.quadTo({0x1.43318p+9,0x1.029eeep+9}, {0x1.477c1cp+9,0x1.035f28p+9});
b.quadTo({0x1.4bbbdap+9,0x1.041d7ap+9}, {0x1.50b6e6p+9,0x1.038298p+9});
b.quadTo({0x1.55e63p+9,0x1.02e15cp+9}, {0x1.5b75dp+9,0x1.00f342p+9});
b.quadTo({0x1.60f6bep+9,0x1.fe1486p+8}, {0x1.667374p+9,0x1.f706ecp+8});
b.quadTo({0x1.6be7f6p+9,0x1.f003ep+8}, {0x1.70186ap+9,0x1.e6f964p+8});
b.quadTo({0x1.74029ep+9,0x1.de8688p+8}, {0x1.75534p+9,0x1.d70c2cp+8});
b.quadTo({0x1.76b608p+9,0x1.cf2a8ep+8}, {0x1.76c3d6p+9,0x1.c5e04ap+8});
b.quadTo({0x1.76d2aep+9,0x1.bbe25ap+8}, {0x1.75dd7ep+9,0x1.b12c28p+8});
b.quadTo({0x1.74f346p+9,0x1.a6f0b8p+8}, {0x1.7302b4p+9,0x1.9f2422p+8});
b.quadTo({0x1.71176ap+9,0x1.976ccap+8}, {0x1.6e0e4cp+9,0x1.9117aep+8});
b.quadTo({0x1.6ad6c6p+9,0x1.8a61c6p+8}, {0x1.669ed8p+9,0x1.84867ep+8});
b.quadTo({0x1.622e86p+9,0x1.7e5ceap+8}, {0x1.5e7a24p+9,0x1.7a2e76p+8});
b.quadTo({0x1.5ab122p+9,0x1.75e8b8p+8}, {0x1.5737cep+9,0x1.727c6cp+8});
b.cubicTo({0x1.5737cep+9,0x1.727c6cp+8}, {0x1.54be18p+9,0x1.6a0c0ap+8}, {0x1.53b912p+9,0x1.6a0c0ap+8});
b.cubicTo({0x1.54f14ap+9,0x1.65189cp+8}, {0x1.562982p+9,0x1.5e1b26p+8}, {0x1.59297ap+9,0x1.5e1b26p+8});
b.quadTo({0x1.5f5cdep+9,0x1.643748p+8}, {0x1.63645cp+9,0x1.68c38ap+8});
b.quadTo({0x1.67807ap+9,0x1.6d6716p+8}, {0x1.6c52a8p+9,0x1.741882p+8});
b.quadTo({0x1.715d3ap+9,0x1.7b183ap+8}, {0x1.754634p+9,0x1.834052p+8});
b.quadTo({0x1.795d96p+9,0x1.8bc936p+8}, {0x1.7bf6ccp+9,0x1.963bdep+8});
b.quadTo({0x1.7e8abap+9,0x1.a09948p+8}, {0x1.7fb582p+9,0x1.ada6d8p+8});
b.quadTo({0x1.80d552p+9,0x1.ba39a6p+8}, {0x1.80c3aap+9,0x1.c61bb6p+8});
b.quadTo({0x1.80b0f8p+9,0x1.d2b172p+8}, {0x1.7ec24p+9,0x1.ddaed4p+8});
b.quadTo({0x1.7cc162p+9,0x1.e91378p+8}, {0x1.776e16p+9,0x1.f4919cp+8});
b.quadTo({0x1.72610ap+9,0x1.ff782p+8}, {0x1.6bdb8cp+9,0x1.03ed0ap+9});
b.quadTo({0x1.655e42p+9,0x1.0818bcp+9}, {0x1.5ebd3p+9,0x1.0a65bep+9});
b.quadTo({0x1.582adp+9,0x1.0cada4p+9}, {0x1.51eb9ap+9,0x1.0d6fe8p+9});
b.quadTo({0x1.4b7826p+9,0x1.0e3886p+9}, {0x1.45c2e4p+9,0x1.0d38d8p+9});
b.quadTo({0x1.40188p+9,0x1.0c3b12p+9}, {0x1.39d0cp+9,0x1.093196p+9});
b.quadTo({0x1.339c12p+9,0x1.063152p+9}, {0x1.2ca538p+9,0x1.006288p+9});
b.quadTo({0x1.25b97cp+9,0x1.f53a0ep+8}, {0x1.1f1f62p+9,0x1.e3faa8p+8});
b.quadTo({0x1.18814ep+9,0x1.d2b0dap+8}, {0x1.1426fep+9,0x1.bf3f18p+8});
b.quadTo({0x1.0fbf34p+9,0x1.ab911ap+8}, {0x1.0e8664p+9,0x1.99f2bap+8});
b.quadTo({0x1.0d49bp+9,0x1.881c46p+8}, {0x1.0e936cp+9,0x1.7a9b4ap+8});
b.quadTo({0x1.0fdf86p+9,0x1.6d019p+8}, {0x1.12d6aep+9,0x1.62f998p+8});
b.quadTo({0x1.159e82p+9,0x1.5991b8p+8}, {0x1.18eb58p+9,0x1.524baep+8});
b.quadTo({0x1.1c1baep+9,0x1.4b4472p+8}, {0x1.1ef16p+9,0x1.468618p+8});
b.quadTo({0x1.21c64ap+9,0x1.41c90ap+8}, {0x1.2761e2p+9,0x1.3abc88p+8});
b.cubicTo({0x1.2761e2p+9,0x1.3abc88p+8}, {0x1.29b804p+9,0x1.393582p+8}, {0x1.2cced2p+9,0x1.393582p+8});
b.cubicTo({0x1.2e46bcp+9,0x1.3de1c6p+8}, {0x1.2fbea8p+9,0x1.48bba4p+8}, {0x1.2f0a3ep+9,0x1.48bba4p+8});
b.close();
SkPath left(b);
b.reset();
b.moveTo({0x1.2cee94p+9,0x1.b0d972p+8});
b.quadTo({0x1.30d7acp+9,0x1.a625ecp+8}, {0x1.325f6ap+9,0x1.a1b576p+8});
b.quadTo({0x1.33f1ap+9,0x1.9d269ep+8}, {0x1.35dfd8p+9,0x1.9790c8p+8});
b.quadTo({0x1.37ee56p+9,0x1.919d88p+8}, {0x1.3a26b6p+9,0x1.8c498ep+8});
b.quadTo({0x1.3c8a1p+9,0x1.868e68p+8}, {0x1.3ebf2cp+9,0x1.82f936p+8});
b.quadTo({0x1.4151eep+9,0x1.7ecbfep+8}, {0x1.45031cp+9,0x1.7c7d18p+8});
b.quadTo({0x1.4904ccp+9,0x1.79fbdcp+8}, {0x1.4cf898p+9,0x1.7b98a8p+8});
b.quadTo({0x1.50c5c2p+9,0x1.7d25b2p+8}, {0x1.536cd4p+9,0x1.812844p+8});
b.quadTo({0x1.559c82p+9,0x1.84765ap+8}, {0x1.5803dcp+9,0x1.8a1d9cp+8});
b.quadTo({0x1.5a33b2p+9,0x1.8f425p+8}, {0x1.5cda52p+9,0x1.96b3cep+8});
b.quadTo({0x1.5f560ap+9,0x1.9dacd8p+8}, {0x1.62dedp+9,0x1.a7000ep+8});
b.quadTo({0x1.663d7ep+9,0x1.afe434p+8}, {0x1.6ba1eep+9,0x1.baa47cp+8});
b.quadTo({0x1.710152p+9,0x1.c55aaep+8}, {0x1.77c024p+9,0x1.cf4818p+8});
b.quadTo({0x1.7e9138p+9,0x1.d95062p+8}, {0x1.8619fep+9,0x1.e1f228p+8});
b.quadTo({0x1.8dbc02p+9,0x1.eab0d8p+8}, {0x1.95a85ap+9,0x1.f25f68p+8});
b.quadTo({0x1.9d8328p+9,0x1.f9fcf2p+8}, {0x1.a4a97cp+9,0x1.ff3e18p+8});
b.quadTo({0x1.abcfa2p+9,0x1.023f8ep+9}, {0x1.b2ace6p+9,0x1.04051ep+9});
b.quadTo({0x1.b99a6p+9,0x1.05cedep+9}, {0x1.bf3c1ep+9,0x1.06d3fep+9});
b.quadTo({0x1.c4e942p+9,0x1.07db2ep+9}, {0x1.c8721ap+9,0x1.084bcep+9});
b.quadTo({0x1.cbfd72p+9,0x1.08bcbcp+9}, {0x1.cdf622p+9,0x1.08e15cp+9});
b.quadTo({0x1.cee07ap+9,0x1.08f25cp+9}, {0x1.cfc68ep+9,0x1.0892cp+9});
b.quadTo({0x1.cf8524p+9,0x1.08adeep+9}, {0x1.cf7992p+9,0x1.08d7c8p+9});
b.quadTo({0x1.cfe046p+9,0x1.076474p+9}, {0x1.cfbf32p+9,0x1.0376d6p+9});
b.quadTo({0x1.cf9b54p+9,0x1.fe694p+8}, {0x1.ce41ep+9,0x1.f172dp+8});
b.quadTo({0x1.ccdc26p+9,0x1.e4068p+8}, {0x1.cadf04p+9,0x1.d665c2p+8});
b.quadTo({0x1.c8d304p+9,0x1.c85f22p+8}, {0x1.c75308p+9,0x1.bdee8p+8});
b.quadTo({0x1.c5c9aap+9,0x1.b33c92p+8}, {0x1.c50992p+9,0x1.ac9b18p+8});
b.quadTo({0x1.c4562ap+9,0x1.a669ap+8}, {0x1.c3c7e8p+9,0x1.a22658p+8});
b.quadTo({0x1.c363dap+9,0x1.9f26d4p+8}, {0x1.c27726p+9,0x1.9c1d2p+8});
b.quadTo({0x1.c1bb02p+9,0x1.99b302p+8}, {0x1.bf19fep+9,0x1.962bb6p+8});
b.quadTo({0x1.bc08eap+9,0x1.920e06p+8}, {0x1.b5e066p+9,0x1.8d601cp+8});
b.quadTo({0x1.af9436p+9,0x1.889716p+8}, {0x1.a8944cp+9,0x1.85d666p+8});
b.quadTo({0x1.a1d42cp+9,0x1.832ecep+8}, {0x1.9dd89p+9,0x1.83c848p+8});
b.quadTo({0x1.9adb04p+9,0x1.843b84p+8}, {0x1.99d0f6p+9,0x1.860ee6p+8});
b.quadTo({0x1.9815a8p+9,0x1.8919a6p+8}, {0x1.964476p+9,0x1.8f00b8p+8});
b.quadTo({0x1.941d6cp+9,0x1.95fe9cp+8}, {0x1.918a0ap+9,0x1.9f4338p+8});
b.quadTo({0x1.8eeab4p+9,0x1.a8b2dap+8}, {0x1.8c6b16p+9,0x1.b1d1f4p+8});
b.quadTo({0x1.89ea4cp+9,0x1.baf55ap+8}, {0x1.881e82p+9,0x1.c18d3ep+8});
b.quadTo({0x1.8659dp+9,0x1.c80b1ep+8}, {0x1.854bbep+9,0x1.cc299ap+8});
b.quadTo({0x1.844a78p+9,0x1.d01622p+8}, {0x1.834aa8p+9,0x1.d4c28cp+8});
b.quadTo({0x1.81eefcp+9,0x1.db1c9ep+8}, {0x1.7fa662p+9,0x1.e08f38p+8});
b.quadTo({0x1.7cd7bcp+9,0x1.e74192p+8}, {0x1.74a244p+9,0x1.ebce0ep+8});
b.cubicTo({0x1.74a244p+9,0x1.ebce0ep+8}, {0x1.71f978p+9,0x1.ea2826p+8}, {0x1.6f379ap+9,0x1.ea2826p+8});
b.cubicTo({0x1.6e7af8p+9,0x1.e4d68ap+8}, {0x1.6dbe58p+9,0x1.da0132p+8}, {0x1.6f4deep+9,0x1.da0132p+8});
b.quadTo({0x1.772d44p+9,0x1.d5a46ep+8}, {0x1.77fc9ep+9,0x1.d3b5c8p+8});
b.quadTo({0x1.795204p+9,0x1.d08762p+8}, {0x1.7a18d8p+9,0x1.cce574p+8});
b.quadTo({0x1.7b3b88p+9,0x1.c795dep+8}, {0x1.7c6542p+9,0x1.c30b66p+8});
b.quadTo({0x1.7d823p+9,0x1.beb2e2p+8}, {0x1.7f567ep+9,0x1.b7fbc2p+8});
b.quadTo({0x1.8123b4p+9,0x1.b15ea6p+8}, {0x1.83a5eap+9,0x1.a8360cp+8});
b.quadTo({0x1.86294cp+9,0x1.9f0926p+8}, {0x1.88cc76p+9,0x1.958bc8p+8});
b.quadTo({0x1.8b7b94p+9,0x1.8be364p+8}, {0x1.8dc08ap+9,0x1.848448p+8});
b.quadTo({0x1.905b58p+9,0x1.7c0e5ap+8}, {0x1.93378ap+9,0x1.77081ap+8});
b.quadTo({0x1.96c4fcp+9,0x1.70ca7cp+8}, {0x1.9d187p+9,0x1.6fd6b8p+8});
b.quadTo({0x1.a26dd4p+9,0x1.6f0932p+8}, {0x1.aa8234p+9,0x1.72369ap+8});
b.quadTo({0x1.b256cap+9,0x1.754aeap+8}, {0x1.b96d9ap+9,0x1.7aade4p+8});
b.quadTo({0x1.c0a816p+9,0x1.802bfap+8}, {0x1.c4ac82p+9,0x1.85904ap+8});
b.quadTo({0x1.c920fep+9,0x1.8b8afep+8}, {0x1.cb01dap+9,0x1.91b6ep+8});
b.quadTo({0x1.ccb226p+9,0x1.97432cp+8}, {0x1.cd7518p+9,0x1.9d1aa8p+8});
b.quadTo({0x1.ce0dd6p+9,0x1.a1ae6p+8}, {0x1.ceca6ep+9,0x1.a830e8p+8});
b.quadTo({0x1.cf7a56p+9,0x1.ae436ep+8}, {0x1.d0ef78p+9,0x1.b8688p+8});
b.quadTo({0x1.d26dfcp+9,0x1.c2cedep+8}, {0x1.d4787cp+9,0x1.d0cb3ep+8});
b.quadTo({0x1.d691dap+9,0x1.df2d8p+8}, {0x1.d80c2p+9,0x1.ed5f3p+8});
b.quadTo({0x1.d992acp+9,0x1.fc06cp+8}, {0x1.d9bdcep+9,0x1.0322aap+9});
b.quadTo({0x1.d9ebbap+9,0x1.08968cp+9}, {0x1.d91ceep+9,0x1.0b8238p+9});
b.quadTo({0x1.d7dbdcp+9,0x1.100b12p+9}, {0x1.d39cf2p+9,0x1.11cecp+9});
b.quadTo({0x1.d08586p+9,0x1.1317a4p+9}, {0x1.cd3cdep+9,0x1.12daa4p+9});
b.quadTo({0x1.cb028ep+9,0x1.12b144p+9}, {0x1.c735e6p+9,0x1.123832p+9});
b.quadTo({0x1.c366bep+9,0x1.11bed2p+9}, {0x1.bd73e2p+9,0x1.10ab02p+9});
b.quadTo({0x1.b775ap+9,0x1.0f9522p+9}, {0x1.b02d1ap+9,0x1.0db3e2p+9});
b.quadTo({0x1.a8d45ep+9,0x1.0bce72p+9}, {0x1.a13684p+9,0x1.0901f4p+9});
b.quadTo({0x1.9998d8p+9,0x1.063588p+9}, {0x1.914ba6p+9,0x1.022f4cp+9});
b.quadTo({0x1.890ffep+9,0x1.fc6328p+8}, {0x1.812182p+9,0x1.f34cd8p+8});
b.quadTo({0x1.7919c8p+9,0x1.ea199ep+8}, {0x1.71d2dcp+9,0x1.df63e8p+8});
b.quadTo({0x1.6a79aep+9,0x1.d49352p+8}, {0x1.649292p+9,0x1.c8ce84p+8});
b.quadTo({0x1.5eb082p+9,0x1.bd13ccp+8}, {0x1.5ae6bp+9,0x1.b314f2p+8});
b.quadTo({0x1.5746f6p+9,0x1.a98528p+8}, {0x1.54b52ep+9,0x1.a24e32p+8});
b.quadTo({0x1.524e4ep+9,0x1.9b8fbp+8}, {0x1.5065a4p+9,0x1.971264p+8});
b.quadTo({0x1.4eb47ep+9,0x1.9317a6p+8}, {0x1.4d652cp+9,0x1.911cbcp+8});
b.quadTo({0x1.4c8d3ep+9,0x1.8fd64ep+8}, {0x1.4af8e8p+9,0x1.8f3158p+8});
b.quadTo({0x1.498b34p+9,0x1.8e9c24p+8}, {0x1.47fee4p+9,0x1.8f93e8p+8});
b.quadTo({0x1.462212p+9,0x1.90be02p+8}, {0x1.450c54p+9,0x1.9280cap+8});
b.quadTo({0x1.4398fp+9,0x1.94db98p+8}, {0x1.41d54ap+9,0x1.991772p+8});
b.quadTo({0x1.3fe6aap+9,0x1.9dba78p+8}, {0x1.3e19a8p+9,0x1.a2f038p+8});
b.quadTo({0x1.3c2c6p+9,0x1.a88362p+8}, {0x1.3a9b16p+9,0x1.ad0f8ap+8});
b.quadTo({0x1.38ff54p+9,0x1.b1ba14p+8}, {0x1.35016cp+9,0x1.bca68ep+8});
b.cubicTo({0x1.35016cp+9,0x1.bca68ep+8}, {0x1.336092p+9,0x1.c2148ap+8}, {0x1.303f16p+9,0x1.c2148ap+8});
b.cubicTo({0x1.2e04b8p+9,0x1.bed2d6p+8}, {0x1.2bca5cp+9,0x1.b54e2ap+8}, {0x1.2b4dbcp+9,0x1.b54e2ap+8});
b.close();
    testPathOp(reporter, left, b, kIntersect_SkPathOp, filename);
}

static void pentrek4(skiatest::Reporter* reporter, const char* filename) {
SkPath b;
b.moveTo(0x1.d8dfb2p+8, 0x1.8cc12ap+7);
b.quadTo(0x1.d393f8p+8, 0x1.9e5cbep+7, 0x1.d1d146p+8, 0x1.a5901ep+7);
b.quadTo(0x1.d00c96p+8, 0x1.accbaap+7, 0x1.cde8c6p+8, 0x1.b7c716p+7);
b.quadTo(0x1.cbdc68p+8, 0x1.c24a3ep+7, 0x1.ca35c4p+8, 0x1.cfe4bp+7);
b.quadTo(0x1.c89fccp+8, 0x1.dcf5c8p+7, 0x1.c833d4p+8, 0x1.ede2d6p+7);
b.quadTo(0x1.c7c7eap+8, 0x1.fecdep+7, 0x1.c916b2p+8, 0x1.08ba84p+8);
b.quadTo(0x1.ca52fap+8, 0x1.118a18p+8, 0x1.cebbdcp+8, 0x1.1b20eep+8);
b.quadTo(0x1.d330b8p+8, 0x1.24d1dp+8, 0x1.daf9bap+8, 0x1.2e0c4cp+8);
b.quadTo(0x1.e2942ap+8, 0x1.370f92p+8, 0x1.ec7eep+8, 0x1.3d1302p+8);
b.quadTo(0x1.f69daap+8, 0x1.433606p+8, 0x1.018452p+9, 0x1.469a7ep+8);
b.quadTo(0x1.07c722p+9, 0x1.4a063ep+8, 0x1.0f999ap+9, 0x1.4a2c1ep+8);
b.quadTo(0x1.179db6p+9, 0x1.4a52ecp+8, 0x1.200a82p+9, 0x1.476966p+8);
b.quadTo(0x1.286ed6p+9, 0x1.4482cep+8, 0x1.2dcf4cp+9, 0x1.4066bp+8);
b.quadTo(0x1.32a8d2p+9, 0x1.3cb1b6p+8, 0x1.34263cp+9, 0x1.39c036p+8);
b.quadTo(0x1.35044ap+9, 0x1.380988p+8, 0x1.351308p+9, 0x1.35fed6p+8);
b.quadTo(0x1.352dc6p+9, 0x1.324a92p+8, 0x1.340786p+9, 0x1.2c6f7cp+8);
b.quadTo(0x1.32d97ep+9, 0x1.266cc4p+8, 0x1.2e5e74p+9, 0x1.1d3b7ep+8);
b.quadTo(0x1.2990d4p+9, 0x1.1360ccp+8, 0x1.21c1dep+9, 0x1.0925f6p+8);
b.quadTo(0x1.1a03bcp+9, 0x1.fe0254p+7, 0x1.10c0ep+9, 0x1.f14a92p+7);
b.quadTo(0x1.074c28p+9, 0x1.e44e58p+7, 0x1.feb2bp+8, 0x1.df142cp+7);
b.quadTo(0x1.ef22c4p+8, 0x1.d9f63p+7, 0x1.e8749p+8, 0x1.daeccp+7);
b.quadTo(0x1.e31764p+8, 0x1.dbb2b8p+7, 0x1.e1b5f8p+8, 0x1.dd6fcp+7);
b.quadTo(0x1.dfc076p+8, 0x1.dfe742p+7, 0x1.dc5932p+8, 0x1.e8ddaap+7);
b.quadTo(0x1.d84a4ap+8, 0x1.f38d86p+7, 0x1.d256d6p+8, 0x1.049736p+8);
b.cubicTo(0x1.d256d6p+8, 0x1.049736p+8, 0x1.cfada8p+8, 0x1.0b3104p+8, 0x1.c997a6p+8, 0x1.0b3104p+8);
b.cubicTo(0x1.c4c1cap+8, 0x1.0887d6p+8, 0x1.bfebeep+8, 0x1.ff914ap+7, 0x1.be27fcp+8, 0x1.ff914ap+7);
b.quadTo(0x1.c779b6p+8, 0x1.ddb27ap+7, 0x1.cc6bcep+8, 0x1.d0ac56p+7);
b.quadTo(0x1.d2058ap+8, 0x1.c1ecbep+7, 0x1.d70e08p+8, 0x1.bb964p+7);
b.quadTo(0x1.dcaa9cp+8, 0x1.b48548p+7, 0x1.e7047p+8, 0x1.b3074p+7);
b.quadTo(0x1.f00d3cp+8, 0x1.b1b9dp+7, 0x1.00f8a8p+9, 0x1.b79bd4p+7);
b.quadTo(0x1.09bfd8p+9, 0x1.bd61a8p+7, 0x1.14002p+9, 0x1.cb756ep+7);
b.quadTo(0x1.1e7244p+9, 0x1.d9cdacp+7, 0x1.273ca2p+9, 0x1.f0d614p+7);
b.quadTo(0x1.2ff62cp+9, 0x1.03d934p+8, 0x1.35878cp+9, 0x1.0f4582p+8);
b.quadTo(0x1.3b6b82p+9, 0x1.1b5b3cp+8, 0x1.3d567ap+9, 0x1.252084p+8);
b.quadTo(0x1.3f493ap+9, 0x1.2f0d6ep+8, 0x1.3f0ef8p+9, 0x1.371f2ap+8);
b.quadTo(0x1.3ec8b6p+9, 0x1.40da78p+8, 0x1.3b2d44p+9, 0x1.47facap+8);
b.quadTo(0x1.38312ep+9, 0x1.4de04ap+8, 0x1.316134p+9, 0x1.53155p+8);
b.quadTo(0x1.2b182ap+9, 0x1.57e332p+8, 0x1.21be7ep+9, 0x1.5b1e9ap+8);
b.quadTo(0x1.186d4ap+9, 0x1.5e5714p+8, 0x1.0f8166p+9, 0x1.5e2be2p+8);
b.quadTo(0x1.0663dep+9, 0x1.5dffc2p+8, 0x1.fdc35ap+8, 0x1.59e582p+8);
b.quadTo(0x1.eea456p+8, 0x1.55c3fap+8, 0x1.e2202p+8, 0x1.4e2cfep+8);
b.quadTo(0x1.d567d6p+8, 0x1.46766ep+8, 0x1.cbb046p+8, 0x1.3af1b4p+8);
b.quadTo(0x1.c22748p+8, 0x1.2fa43p+8, 0x1.bc9024p+8, 0x1.237c12p+8);
b.quadTo(0x1.b6ed06p+8, 0x1.1739e8p+8, 0x1.b5484ep+8, 0x1.0b817cp+8);
b.quadTo(0x1.b3b616p+8, 0x1.004d1p+8, 0x1.b43a2cp+8, 0x1.ebe52ap+7);
b.quadTo(0x1.b4be34p+8, 0x1.d73238p+7, 0x1.b6c63cp+8, 0x1.c6755p+7);
b.quadTo(0x1.b8bd98p+8, 0x1.b641c2p+7, 0x1.bb463ap+8, 0x1.a940eap+7);
b.quadTo(0x1.bdb76ap+8, 0x1.9cb856p+7, 0x1.bfd9bap+8, 0x1.93fde2p+7);
b.quadTo(0x1.c1fe08p+8, 0x1.8b3b42p+7, 0x1.c7bc4ep+8, 0x1.7822d6p+7);
b.cubicTo(0x1.c7bc4ep+8, 0x1.7822d6p+7, 0x1.ca949ap+8, 0x1.6b9e04p+7, 0x1.d0bad8p+8, 0x1.6b9e04p+7);
b.cubicTo(0x1.d57594p+8, 0x1.714e9cp+7, 0x1.da305p+8, 0x1.834bb2p+7, 0x1.dbb7fep+8, 0x1.834bb2p+7);
b.close();
SkPath left(b);
b.reset();
b.moveTo(0x1.9c6034p+8, 0x1.1e47bp+8);
b.quadTo(0x1.a8507ap+8, 0x1.27ffbcp+8, 0x1.b0e098p+8, 0x1.2dd55ap+8);
b.cubicTo(0x1.b0e098p+8, 0x1.2dd55ap+8, -0x1.670b34p+10, 0x1.8a539cp+10, 0x1.1f92e8p+11, 0x1.8a539cp+10);
b.quadTo(0x1.b9601ap+8, 0x1.33a304p+8, 0x1.c5bed8p+8, 0x1.3a7c7ap+8);
b.cubicTo(0x1.c5bed8p+8, 0x1.3a7c7ap+8, 0x1.d06b0ap+10, -0x1.d0606ep+8, -0x1.daabd4p+9, -0x1.d0606ep+8);
b.quadTo(0x1.d207bp+8, 0x1.414e32p+8, 0x1.e587f4p+8, 0x1.49a596p+8);
b.cubicTo(0x1.e587f4p+8, 0x1.49a596p+8, -0x1.957bccp+8, 0x1.63dd74p+9, 0x1.58095p+10, 0x1.63dd74p+9);
b.quadTo(0x1.f90bb6p+8, 0x1.5204d2p+8, 0x1.08ea28p+9, 0x1.5a4778p+8);
b.cubicTo(0x1.08ea28p+9, 0x1.5a4778p+8, -0x1.294f56p+12, 0x1.0843bap+11, 0x1.6b6b56p+12, 0x1.0843bap+11);
b.quadTo(0x1.152cfcp+9, 0x1.627b66p+8, 0x1.22734cp+9, 0x1.67f912p+8);
b.cubicTo(0x1.22734cp+9, 0x1.67f912p+8, -0x1.bf0342p+9, 0x1.4d5148p+9, 0x1.00f014p+11, 0x1.4d5148p+9);
b.quadTo(0x1.2f97d2p+9, 0x1.6d7076p+8, 0x1.3dfba2p+9, 0x1.6ef374p+8);
b.cubicTo(0x1.3dfba2p+9, 0x1.6ef374p+8, 0x1.28ce1p+11, 0x1.11b81p+8, -0x1.139944p+10, 0x1.11b81p+8);
b.quadTo(0x1.4c4356p+9, 0x1.707b94p+8, 0x1.57d58cp+9, 0x1.6da4cep+8);
b.cubicTo(0x1.57d58cp+9, 0x1.6da4cep+8, 0x1.438e82p+8, 0x1.4160fcp+8, 0x1.06f5c6p+10, 0x1.4160fcp+8);
b.quadTo(0x1.6320c2p+9, 0x1.6ae5f4p+8, 0x1.6b49bcp+9, 0x1.642a54p+8);
b.cubicTo(0x1.6b49bcp+9, 0x1.642a54p+8, -0x1.2c52ep+11, -0x1.d1acep+9, 0x1.e2256p+11, -0x1.d1acep+9);
b.quadTo(0x1.73639ep+9, 0x1.5d807ap+8, 0x1.791942p+9, 0x1.545e7ep+8);
b.cubicTo(0x1.791942p+9, 0x1.545e7ep+8, -0x1.73b9d4p+11, -0x1.49a80ep+11, 0x1.18560ap+12, -0x1.49a80ep+11);
b.quadTo(0x1.7e6c26p+9, 0x1.4bdf38p+8, 0x1.80a8f6p+9, 0x1.421f3cp+8);
b.quadTo(0x1.82e4dcp+9, 0x1.3869ap+8, 0x1.82a8cep+9, 0x1.2b12c4p+8);
b.quadTo(0x1.826e02p+9, 0x1.1da4c4p+8, 0x1.7dbb8cp+9, 0x1.0b5108p+8);
b.cubicTo(0x1.7dbb8cp+9, 0x1.0b5108p+8, 0x1.2aba3ap+10, -0x1.1f69c4p+9, 0x1.4cdddep+8, -0x1.1f69c4p+9);
b.quadTo(0x1.78f64ep+9, 0x1.f15052p+7, 0x1.6defdcp+9, 0x1.c8c4e2p+7);
b.cubicTo(0x1.6defdcp+9, 0x1.c8c4e2p+7, 0x1.231d64p+10, -0x1.539222p+7, 0x1.2bb3cep+8, -0x1.539222p+7);
b.quadTo(0x1.62c68cp+9, 0x1.9fa2d6p+7, 0x1.5651a6p+9, 0x1.851544p+7);
b.cubicTo(0x1.5651a6p+9, 0x1.851544p+7, -0x1.49f63cp+9, 0x1.c8406ep+9, 0x1.fb1b04p+10, 0x1.c8406ep+9);
b.quadTo(0x1.49ed84p+9, 0x1.6a99fep+7, 0x1.417218p+9, 0x1.62a7aap+7);
b.cubicTo(0x1.417218p+9, 0x1.62a7aap+7, -0x1.42ccdp+8, 0x1.94848ap+8, 0x1.921572p+10, 0x1.94848ap+8);
b.quadTo(0x1.39405ep+9, 0x1.5af11ap+7, 0x1.3507dap+9, 0x1.5d2f38p+7);
b.cubicTo(0x1.3507dap+9, 0x1.5d2f38p+7, 0x1.5a940ap+10, 0x1.13df3ap+8, -0x1.2c9aep+7, 0x1.13df3ap+8);
b.quadTo(0x1.315d12p+9, 0x1.5f1e3p+7, 0x1.2fe356p+9, 0x1.64173cp+7);
b.cubicTo(0x1.2fe356p+9, 0x1.64173cp+7, -0x1.45b8c2p+9, -0x1.b8c97cp+9, 0x1.d30768p+10, -0x1.b8c97cp+9);
b.quadTo(0x1.2e072p+9, 0x1.6a58bcp+7, 0x1.2c84fp+9, 0x1.758a2ap+7);
b.cubicTo(0x1.2c84fp+9, 0x1.758a2ap+7, -0x1.ab989p+7, -0x1.4aef8ep+10, 0x1.626a2ep+10, -0x1.4aef8ep+10);
b.quadTo(0x1.2abea8p+9, 0x1.82ab5ap+7, 0x1.29244ap+9, 0x1.94dd0ep+7);
b.cubicTo(0x1.29244ap+9, 0x1.94dd0ep+7, 0x1.f16ff8p+8, -0x1.20f58p+6, 0x1.59bb4p+9, -0x1.20f58p+6);
b.quadTo(0x1.277dbep+9, 0x1.a7863ap+7, 0x1.2653d8p+9, 0x1.baa55p+7);
b.cubicTo(0x1.2653d8p+9, 0x1.baa55p+7, 0x1.4d11f6p+9, 0x1.0dc622p+9, 0x1.fec3b8p+8, 0x1.0dc622p+9);
b.quadTo(0x1.2525fcp+9, 0x1.cdea6ep+7, 0x1.24649ep+9, 0x1.e0a46cp+7);
b.cubicTo(0x1.24649ep+9, 0x1.e0a46cp+7, 0x1.fa8836p+8, -0x1.e930bap+7, 0x1.4bd5d4p+9, -0x1.e930bap+7);
b.quadTo(0x1.23a3a6p+9, 0x1.f32d5ep+7, 0x1.235ce6p+9, 0x1.01cebp+8);
b.cubicTo(0x1.235ce6p+9, 0x1.01cebp+8, 0x1.558dd4p+9, 0x1.b5903cp+10, 0x1.e05ad6p+8, 0x1.b5903cp+10);
b.quadTo(0x1.2314e4p+9, 0x1.0a0222p+8, 0x1.23264cp+9, 0x1.0f7132p+8);
b.cubicTo(0x1.23264cp+9, 0x1.0f7132p+8, 0x1.2cb034p+9, -0x1.eb1e2cp+8, 0x1.1a2902p+9, -0x1.eb1e2cp+8);
b.quadTo(0x1.233716p+9, 0x1.15016p+8, 0x1.236002p+9, 0x1.18903ap+8);
b.cubicTo(0x1.236002p+9, 0x1.18903ap+8, 0x1.157016p+9, 0x1.2767bp+9, 0x1.311022p+9, 0x1.2767bp+9);
b.quadTo(0x1.23895p+9, 0x1.1c3856p+8, 0x1.240be6p+9, 0x1.23c09ap+8);
b.cubicTo(0x1.240be6p+9, 0x1.23c09ap+8, 0x1.246ab6p+9, 0x1.2e4364p+8, 0x1.227f54p+9, 0x1.2e4364p+8);
b.cubicTo(0x1.1fc346p+9, 0x1.2f0106p+8, 0x1.1d0736p+9, 0x1.2be7e6p+8, 0x1.1a81ep+9, 0x1.2be7e6p+8);
b.quadTo(0x1.199878p+9, 0x1.1e7142p+8, 0x1.196a1ap+9, 0x1.1a565ep+8);
b.lineTo(0x1.196a4ap+9, 0x1.1a5a7ap+8);
b.quadTo(0x1.1939cap+9, 0x1.162328p+8, 0x1.19270ep+9, 0x1.0fed9p+8);
b.lineTo(0x1.192718p+9, 0x1.0ff13ep+8);
b.quadTo(0x1.1912b4p+9, 0x1.0992bap+8, 0x1.1962c8p+9, 0x1.0073b8p+8);
b.lineTo(0x1.1962aap+9, 0x1.00772p+8);
b.quadTo(0x1.19b182p+9, 0x1.ee9d1ep+7, 0x1.1a8592p+9, 0x1.da3ed2p+7);
b.lineTo(0x1.1a854ep+9, 0x1.da4568p+7);
b.quadTo(0x1.1b57p+9, 0x1.c5f666p+7, 0x1.1c9cdep+9, 0x1.b12912p+7);
b.lineTo(0x1.1c9c7cp+9, 0x1.b12f5cp+7);
b.quadTo(0x1.1de512p+9, 0x1.9c1802p+7, 0x1.1fb65cp+9, 0x1.878b84p+7);
b.lineTo(0x1.1fb5d6p+9, 0x1.879172p+7);
b.quadTo(0x1.21930cp+9, 0x1.726916p+7, 0x1.23b84p+9, 0x1.6289ap+7);
b.lineTo(0x1.23b788p+9, 0x1.628ef6p+7);
b.quadTo(0x1.2621ap+9, 0x1.50a4c8p+7, 0x1.2972ep+9, 0x1.457ccap+7);
b.lineTo(0x1.2971d6p+9, 0x1.45804cp+7);
b.quadTo(0x1.2d282ep+9, 0x1.38fd68p+7, 0x1.33b808p+9, 0x1.3587b4p+7);
b.lineTo(0x1.33b6c2p+9, 0x1.35886p+7);
b.quadTo(0x1.39c13ap+9, 0x1.3252aep+7, 0x1.43bb32p+9, 0x1.3bb6b4p+7);
b.lineTo(0x1.43b9e4p+9, 0x1.3bb57ap+7);
b.quadTo(0x1.4d7bf8p+9, 0x1.44d9dap+7, 0x1.5b06dp+9, 0x1.61caf4p+7);
b.lineTo(0x1.5b059ap+9, 0x1.61c86p+7);
b.quadTo(0x1.689bf4p+9, 0x1.7ebef2p+7, 0x1.74b57ap+9, 0x1.ab563ep+7);
b.lineTo(0x1.74b47cp+9, 0x1.ab5292p+7);
b.quadTo(0x1.810cbap+9, 0x1.d8b846p+7, 0x1.86a248p+9, 0x1.0233d6p+8);
b.lineTo(0x1.86a1bp+9, 0x1.023188p+8);
b.quadTo(0x1.8c58f6p+9, 0x1.187eep+8, 0x1.8ca73ap+9, 0x1.2a5eccp+8);
b.quadTo(0x1.8cf8a8p+9, 0x1.3c751cp+8, 0x1.89bf96p+9, 0x1.4a7704p+8);
b.quadTo(0x1.8684d2p+9, 0x1.58899cp+8, 0x1.7f5726p+9, 0x1.63feacp+8);
b.lineTo(0x1.7f5836p+9, 0x1.63fcfap+8);
b.quadTo(0x1.788452p+9, 0x1.6ee8d2p+8, 0x1.6f18cp+9, 0x1.76a86ep+8);
b.lineTo(0x1.6f1a0cp+9, 0x1.76a75cp+8);
b.quadTo(0x1.65af3ep+9, 0x1.7e6c88p+8, 0x1.590bd8p+9, 0x1.817f0ep+8);
b.lineTo(0x1.590d4p+9, 0x1.817eb6p+8);
b.quadTo(0x1.4c9b52p+9, 0x1.848c6p+8, 0x1.3d73f2p+9, 0x1.82ec42p+8);
b.lineTo(0x1.3d755ap+9, 0x1.82ec68p+8);
b.quadTo(0x1.2e4e26p+9, 0x1.8154e2p+8, 0x1.206b7p+9, 0x1.7b8e64p+8);
b.lineTo(0x1.206cdp+9, 0x1.7b8ef6p+8);
b.quadTo(0x1.129154p+9, 0x1.75d396p+8, 0x1.05bf5ep+9, 0x1.6d3fe6p+8);
b.lineTo(0x1.05c0bp+9, 0x1.6d40c8p+8);
b.quadTo(0x1.f1edc2p+8, 0x1.64ba76p+8, 0x1.dda7dep+8, 0x1.5c07fp+8);
b.lineTo(0x1.ddaa64p+8, 0x1.5c0906p+8);
b.quadTo(0x1.c9388p+8, 0x1.534a4ap+8, 0x1.bc0c46p+8, 0x1.4bfa56p+8);
b.lineTo(0x1.bc0ea8p+8, 0x1.4bfbaap+8);
b.quadTo(0x1.aede6ep+8, 0x1.44ae34p+8, 0x1.a59b68p+8, 0x1.3e5b04p+8);
b.lineTo(0x1.a59da4p+8, 0x1.3e5c8ap+8);
b.quadTo(0x1.9c593ep+8, 0x1.380c18p+8, 0x1.8fbfccp+8, 0x1.2dca5p+8);
b.cubicTo(0x1.8fbfccp+8, 0x1.2dca5p+8, 0x1.8b7812p+8, 0x1.240084p+8, 0x1.8ad2aep+8, 0x1.240084p+8);
b.cubicTo(0x1.8e4ebp+8, 0x1.1fb8ccp+8, 0x1.91cab2p+8, 0x1.1acbaep+8, 0x1.98187cp+8, 0x1.1acbaep+8);
b.close();
    testPathOp(reporter, left, b, kIntersect_SkPathOp, filename);
}

static void pentrek5(skiatest::Reporter* reporter, const char* filename) {
SkPath b;
b.moveTo(0x1.24d59p+9, 0x1.a35da6p+7);
b.quadTo(0x1.2fca0cp+9, 0x1.9dbdfcp+7, 0x1.3692d4p+9, 0x1.9d3edcp+7);
b.quadTo(0x1.3d559ep+9, 0x1.9cc02cp+7, 0x1.463532p+9, 0x1.9fa812p+7);
b.quadTo(0x1.4f2882p+9, 0x1.a2967p+7, 0x1.581af6p+9, 0x1.aa4f2ap+7);
b.quadTo(0x1.612508p+9, 0x1.b21c48p+7, 0x1.67c34cp+9, 0x1.bcb5dep+7);
b.quadTo(0x1.6e940ep+9, 0x1.c7a052p+7, 0x1.723cb6p+9, 0x1.d25eb6p+7);
b.quadTo(0x1.764fd4p+9, 0x1.de55b6p+7, 0x1.77cc3p+9, 0x1.e99a88p+7);
b.quadTo(0x1.798272p+9, 0x1.f6966ep+7, 0x1.7867bep+9, 0x1.0a3c56p+8);
b.cubicTo(0x1.7867bep+9, 0x1.0a3c56p+8, 0x1.78007p+9, 0x1.137916p+8, 0x1.75753p+9, 0x1.137916p+8);
b.cubicTo(0x1.72ba54p+9, 0x1.12aa7cp+8, 0x1.6fff7ap+9, 0x1.0cc56p+8, 0x1.6e1bf4p+9, 0x1.0cc56p+8);
b.quadTo(0x1.6f238ep+9, 0x1.fdad92p+7, 0x1.6ef3dp+9, 0x1.fc4378p+7);
b.quadTo(0x1.6e8a2cp+9, 0x1.f9224ap+7, 0x1.6c51cap+9, 0x1.f29d4ap+7);
b.quadTo(0x1.69aef2p+9, 0x1.eadfaep+7, 0x1.640bb4p+9, 0x1.e1d822p+7);
b.quadTo(0x1.5e35f8p+9, 0x1.d87fb8p+7, 0x1.55ff0ap+9, 0x1.d168d6p+7);
b.quadTo(0x1.4db07ep+9, 0x1.ca3d9p+7, 0x1.45644ep+9, 0x1.c785eep+7);
b.quadTo(0x1.3d0462p+9, 0x1.c4c7d4p+7, 0x1.36c1acp+9, 0x1.c53d24p+7);
b.quadTo(0x1.3084f4p+9, 0x1.c5b204p+7, 0x1.261b7p+9, 0x1.cb0a5ap+7);
b.cubicTo(0x1.261b7p+9, 0x1.cb0a5ap+7, 0x1.235ebap+9, 0x1.c4b28ep+7, 0x1.20dcd6p+9, 0x1.c4b28ep+7);
b.cubicTo(0x1.2082eap+9, 0x1.b9bfbcp+7, 0x1.2028fcp+9, 0x1.a4c55ap+7, 0x1.2218dcp+9, 0x1.a4c55ap+7);
b.close();
SkPath left(b);
b.reset();
b.moveTo(0x1.5e8198p+9, 0x1.66dacp+7);
b.quadTo(0x1.627c8ap+9, 0x1.93fcc4p+7, 0x1.662694p+9, 0x1.aa20cap+7);
b.quadTo(0x1.69f236p+9, 0x1.c10fd8p+7, 0x1.6ef93ep+9, 0x1.d740f6p+7);
b.quadTo(0x1.73fb8p+9, 0x1.ed5cfcp+7, 0x1.785fecp+9, 0x1.fb576p+7);
b.quadTo(0x1.7cb1e6p+9, 0x1.048b86p+8, 0x1.7fa99ap+9, 0x1.07951p+8);
b.quadTo(0x1.8292bap+9, 0x1.0a8facp+8, 0x1.8653f4p+9, 0x1.0c77b4p+8);
b.cubicTo(0x1.8653f4p+9, 0x1.0c77b4p+8, 0x1.8900aep+9, 0x1.13457p+8, 0x1.8a9f7ap+9, 0x1.13457p+8);
b.cubicTo(0x1.89f1a6p+9, 0x1.189ee6p+8, 0x1.8943d2p+9, 0x1.2135f2p+8, 0x1.868ac8p+9, 0x1.2135f2p+8);
b.quadTo(0x1.7f0546p+9, 0x1.1d6454p+8, 0x1.7b1b66p+9, 0x1.1962fp+8);
b.quadTo(0x1.77401ap+9, 0x1.15707ap+8, 0x1.722614p+9, 0x1.0d525p+8);
b.quadTo(0x1.6d1e8p+9, 0x1.055182p+8, 0x1.679042p+9, 0x1.f21d0ap+7);
b.quadTo(0x1.6206cap+9, 0x1.d9ac28p+7, 0x1.5dcfecp+9, 0x1.c03536p+7);
b.quadTo(0x1.597776p+9, 0x1.a5f33cp+7, 0x1.551368p+9, 0x1.74294p+7);
b.cubicTo(0x1.551368p+9, 0x1.74294p+7, 0x1.542864p+9, 0x1.5e51a8p+7, 0x1.558678p+9, 0x1.5e51a8p+7);
b.cubicTo(0x1.5820bp+9, 0x1.5aa59cp+7, 0x1.5abae8p+9, 0x1.5c71e4p+7, 0x1.5d9696p+9, 0x1.5c71e4p+7);
b.close();
    testPathOp(reporter, left, b, kIntersect_SkPathOp, filename);
}

static void pentrek6(skiatest::Reporter* reporter, const char* filename) {
SkPath b;
b.moveTo(0x1.de007ep+8, 0x1.a9dep+7);
b.quadTo(0x1.f0ce8p+8, 0x1.a9d852p+7, 0x1.faa614p+8, 0x1.aa1c28p+7);
b.cubicTo(0x1.faa614p+8, 0x1.aa1c28p+7, 0x1.faabfcp+8, 0x1.aa1d86p+7, 0x1.fac84p+8, 0x1.aa1d86p+7);
b.quadTo(0x1.025b34p+9, 0x1.aab1ap+7, 0x1.07268ep+9, 0x1.ad2b5cp+7);
b.cubicTo(0x1.07268ep+9, 0x1.ad2b5cp+7, 0x1.06fffcp+9, 0x1.ad4e34p+7, 0x1.076334p+9, 0x1.ad4e34p+7);
b.quadTo(0x1.0c0c2ep+9, 0x1.b00b36p+7, 0x1.102e3cp+9, 0x1.b5ac4ap+7);
b.cubicTo(0x1.102e3cp+9, 0x1.b5ac4ap+7, 0x1.1026b6p+9, 0x1.b5d6dp+7, 0x1.104c4ap+9, 0x1.b5d6dp+7);
b.quadTo(0x1.141d7cp+9, 0x1.bb5be4p+7, 0x1.177542p+9, 0x1.c26886p+7);
b.cubicTo(0x1.177542p+9, 0x1.c26886p+7, 0x1.176fe6p+9, 0x1.c2a7d8p+7, 0x1.179264p+9, 0x1.c2a7d8p+7);
b.quadTo(0x1.1aa7d8p+9, 0x1.c97f0cp+7, 0x1.1d38acp+9, 0x1.d0c226p+7);
b.cubicTo(0x1.1d38acp+9, 0x1.d0c226p+7, 0x1.1d415ep+9, 0x1.d0f2dp+7, 0x1.1d49a6p+9, 0x1.d0f2dp+7);
b.quadTo(0x1.1fbdc6p+9, 0x1.d84348p+7, 0x1.2151a4p+9, 0x1.de37c8p+7);
b.cubicTo(0x1.2151a4p+9, 0x1.de37c8p+7, 0x1.2155eap+9, 0x1.de8b02p+7, 0x1.216732p+9, 0x1.de8b02p+7);
b.quadTo(0x1.23144p+9, 0x1.e53becp+7, 0x1.257e36p+9, 0x1.f49234p+7);
b.cubicTo(0x1.257e36p+9, 0x1.f49234p+7, 0x1.26f696p+9, 0x1.0522aap+8, 0x1.2642cap+9, 0x1.0522aap+8);
b.cubicTo(0x1.23ecf4p+9, 0x1.08136cp+8, 0x1.21971cp+9, 0x1.099c94p+8, 0x1.1e802ap+9, 0x1.099c94p+8);
b.quadTo(0x1.1b379p+9, 0x1.fe5c48p+7, 0x1.1a5ae8p+9, 0x1.faeb58p+7);
b.lineTo(0x1.1a74ap+9, 0x1.fb4f3p+7);
b.quadTo(0x1.194cdap+9, 0x1.f6f2b4p+7, 0x1.17587p+9, 0x1.f11ef6p+7);
b.lineTo(0x1.1771f4p+9, 0x1.f1691ap+7);
b.quadTo(0x1.1553ap+9, 0x1.eb6a14p+7, 0x1.12b38cp+9, 0x1.e5973cp+7);
b.lineTo(0x1.12cb62p+9, 0x1.e5cacep+7);
b.quadTo(0x1.10106cp+9, 0x1.e008d8p+7, 0x1.0cde78p+9, 0x1.db6a0ap+7);
b.lineTo(0x1.0cf51p+9, 0x1.db89c2p+7);
b.quadTo(0x1.09b08ep+9, 0x1.d7166ap+7, 0x1.05c8a6p+9, 0x1.d4cadcp+7);
b.lineTo(0x1.05ded2p+9, 0x1.d4d718p+7);
b.quadTo(0x1.01919p+9, 0x1.d29e78p+7, 0x1.fa3914p+8, 0x1.d21986p+7);
b.lineTo(0x1.fa6128p+8, 0x1.d21b3cp+7);
b.quadTo(0x1.f0ad8cp+8, 0x1.d1d85ep+7, 0x1.de0382p+8, 0x1.d1dep+7);
b.cubicTo(0x1.de0382p+8, 0x1.d1dep+7, 0x1.d87e9cp+8, 0x1.c8ead2p+7, 0x1.d402d6p+8, 0x1.c8ead2p+7);
b.cubicTo(0x1.d402p+8, 0x1.bde104p+7, 0x1.d4012ap+8, 0x1.a9dfaap+7, 0x1.d87b96p+8, 0x1.a9dfaap+7);
b.close();
SkPath left(b);
b.reset();
b.moveTo(0x1.fd63fap+8, 0x1.c765b6p+7);
b.quadTo(0x1.04860ap+9, 0x1.df4834p+7, 0x1.05e296p+9, 0x1.e308eep+7);
b.lineTo(0x1.057df6p+9, 0x1.e20e3cp+7);
b.quadTo(0x1.06feep+9, 0x1.e56ce4p+7, 0x1.090ffcp+9, 0x1.e8f05ap+7);
b.lineTo(0x1.088b38p+9, 0x1.e8307cp+7);
b.quadTo(0x1.0a4dc6p+9, 0x1.ea4d46p+7, 0x1.0c7b92p+9, 0x1.eafcd8p+7);
b.lineTo(0x1.0bdbe6p+9, 0x1.eaf28ep+7);
b.quadTo(0x1.0d7acep+9, 0x1.eaa5a2p+7, 0x1.0eb7cap+9, 0x1.e850a6p+7);
b.lineTo(0x1.0e2fdap+9, 0x1.e97a22p+7);
b.quadTo(0x1.0fb6d6p+9, 0x1.e5a316p+7, 0x1.11107cp+9, 0x1.e02edap+7);
b.lineTo(0x1.10b854p+9, 0x1.e1b9dcp+7);
b.quadTo(0x1.11f874p+9, 0x1.db80bcp+7, 0x1.12e024p+9, 0x1.d400d4p+7);
b.lineTo(0x1.12b7fap+9, 0x1.d5720ap+7);
b.quadTo(0x1.13b8c6p+9, 0x1.cb221p+7, 0x1.14f9p+9, 0x1.be6878p+7);
b.lineTo(0x1.14d6ap+9, 0x1.bff53p+7);
b.quadTo(0x1.15de1p+9, 0x1.b24b5cp+7, 0x1.173afcp+9, 0x1.a0b11ap+7);
b.cubicTo(0x1.173afcp+9, 0x1.a0b11ap+7, 0x1.180bfcp+9, 0x1.903d0ep+7, 0x1.1ad98cp+9, 0x1.903d0ep+7);
b.cubicTo(0x1.1d7c6p+9, 0x1.938114p+7, 0x1.201f32p+9, 0x1.a1fb58p+7, 0x1.219962p+9, 0x1.a1fb58p+7);
b.quadTo(0x1.1f6e64p+9, 0x1.bdfb0cp+7, 0x1.1e69dp+9, 0x1.cb7efp+7);
b.cubicTo(0x1.1e69dp+9, 0x1.cb7efp+7, 0x1.1e5fdep+9, 0x1.cc8b5ep+7, 0x1.1e540cp+9, 0x1.cc8b5ep+7);
b.quadTo(0x1.1d08e2p+9, 0x1.d9b43p+7, 0x1.1c09c2p+9, 0x1.e3f316p+7);
b.cubicTo(0x1.1c09c2p+9, 0x1.e3f316p+7, 0x1.1bfd9cp+9, 0x1.e4eb9cp+7, 0x1.1bf028p+9, 0x1.e4eb9cp+7);
b.quadTo(0x1.1a7aep+9, 0x1.f100d4p+7, 0x1.1883bp+9, 0x1.fac904p+7);
b.cubicTo(0x1.1883bp+9, 0x1.fac904p+7, 0x1.186816p+9, 0x1.fbd6cep+7, 0x1.184a88p+9, 0x1.fbd6cep+7);
b.quadTo(0x1.16029ap+9, 0x1.0286c6p+8, 0x1.138236p+9, 0x1.05abd6p+8);
b.cubicTo(0x1.138236p+9, 0x1.05abd6p+8, 0x1.1356a8p+9, 0x1.0614cp+8, 0x1.1328d2p+9, 0x1.0614cp+8);
b.quadTo(0x1.0fef5ap+9, 0x1.091e0cp+8, 0x1.0c526ep+9, 0x1.0973cap+8);
b.cubicTo(0x1.0c526ep+9, 0x1.0973cap+8, 0x1.0c1d34p+9, 0x1.0977p+8, 0x1.0be7ep+9, 0x1.0977p+8);
b.quadTo(0x1.087422p+9, 0x1.08ebecp+8, 0x1.05ab74p+9, 0x1.07403ap+8);
b.cubicTo(0x1.05ab74p+9, 0x1.07403ap+8, 0x1.057e3p+9, 0x1.070544p+8, 0x1.05523p+9, 0x1.070544p+8);
b.quadTo(0x1.029538p+9, 0x1.04b176p+8, 0x1.009aa2p+9, 0x1.0279cap+8);
b.cubicTo(0x1.009aa2p+9, 0x1.0279cap+8, 0x1.007852p+9, 0x1.022914p+8, 0x1.005666p+9, 0x1.022914p+8);
b.quadTo(0x1.fc185ep+8, 0x1.fe025cp+7, 0x1.ef1406p+8, 0x1.e3564ap+7);
b.cubicTo(0x1.ef1406p+8, 0x1.e3564ap+7, 0x1.eb391ep+8, 0x1.cec3dap+7, 0x1.eb4cc2p+8, 0x1.cec3dap+7);
b.cubicTo(0x1.ef3fdcp+8, 0x1.c70e0cp+7, 0x1.f332f4p+8, 0x1.bf7f86p+7, 0x1.f98914p+8, 0x1.bf7f86p+7);
b.close();
    testPathOp(reporter, left, b, kIntersect_SkPathOp, filename);
}

static void pentrek7(skiatest::Reporter* reporter, const char* filename) {
SkPath b;
b.moveTo(0x1.35e382p+7, 0x1.4b157p+6);
b.quadTo(0x1.7000c8p+7, 0x1.636ce6p+6, 0x1.b110d6p+7, 0x1.98653p+6);
b.cubicTo(0x1.b110d6p+7, 0x1.98653p+6, 0x1.b150fep+7, 0x1.98e198p+6, 0x1.b1a44p+7, 0x1.98e198p+6);
b.quadTo(0x1.03ddacp+8, 0x1.e82182p+6, 0x1.32e5a4p+8, 0x1.2383bp+7);
b.cubicTo(0x1.32e5a4p+8, 0x1.2383bp+7, 0x1.322e14p+8, 0x1.24642ep+7, 0x1.33bcecp+8, 0x1.24642ep+7);
b.quadTo(0x1.6aa9b6p+8, 0x1.5dd264p+7, 0x1.ab3e12p+8, 0x1.a23452p+7);
b.cubicTo(0x1.ab3e12p+8, 0x1.a23452p+7, 0x1.b01ec2p+8, 0x1.b37b92p+7, 0x1.b1fbccp+8, 0x1.b37b92p+7);
b.cubicTo(0x1.af66bp+8, 0x1.bd3cf4p+7, 0x1.acd192p+8, 0x1.cab868p+7, 0x1.a6c30ep+8, 0x1.cab868p+7);
b.quadTo(0x1.6159c6p+8, 0x1.8138b4p+7, 0x1.29c294p+8, 0x1.471852p+7);
b.lineTo(0x1.29e3acp+8, 0x1.473a5p+7);
b.quadTo(0x1.f65b3ep+7, 0x1.181a06p+7, 0x1.a129bcp+7, 0x1.e1c77cp+6);
b.lineTo(0x1.a1fc12p+7, 0x1.e27ddp+6);
b.quadTo(0x1.644498p+7, 0x1.b03ee2p+6, 0x1.2db07ep+7, 0x1.99629p+6);
b.cubicTo(0x1.2db07ep+7, 0x1.99629p+6, 0x1.22e2b2p+7, 0x1.7fa496p+6, 0x1.1bf37cp+7, 0x1.7fa496p+6);
b.cubicTo(0x1.1e36b8p+7, 0x1.6a08fcp+6, 0x1.2079f4p+7, 0x1.468ef6p+6, 0x1.2b15b4p+7, 0x1.468ef6p+6);
b.close();
SkPath left(b);
b.reset();
b.moveTo(0x1.05afa8p+8, 0x1.d2b1d4p+6);
b.quadTo(0x1.ff9c24p+7, 0x1.0cc8d6p+7, 0x1.ff5dc4p+7, 0x1.23ff1ep+7);
b.lineTo(0x1.fe94b6p+7, 0x1.1e3bb4p+7);
b.quadTo(0x1.025d6p+8, 0x1.33817ep+7, 0x1.0664b8p+8, 0x1.40ef6ep+7);
b.lineTo(0x1.04fa7p+8, 0x1.3d433ap+7);
b.quadTo(0x1.0a9a82p+8, 0x1.48cd18p+7, 0x1.1392bep+8, 0x1.527398p+7);
b.lineTo(0x1.11c7bap+8, 0x1.50f38ep+7);
b.quadTo(0x1.19ce54p+8, 0x1.55e59ap+7, 0x1.1ceedap+8, 0x1.519804p+7);
b.lineTo(0x1.1b1f1ep+8, 0x1.54cbc4p+7);
b.quadTo(0x1.1ca31p+8, 0x1.516164p+7, 0x1.1cb704p+8, 0x1.502154p+7);
b.quadTo(0x1.1caa7cp+8, 0x1.50ea52p+7, 0x1.1cca4p+8, 0x1.51650ap+7);
b.lineTo(0x1.1e29a6p+8, 0x1.55691p+7);
b.quadTo(0x1.1a2aep+8, 0x1.4c542cp+7, 0x1.07b6e2p+8, 0x1.3d8ab6p+7);
b.cubicTo(0x1.07b6e2p+8, 0x1.3d8ab6p+7, 0x1.029756p+8, 0x1.2dc8dcp+7, 0x1.001928p+8, 0x1.2dc8dcp+7);
b.cubicTo(0x1.0226a4p+8, 0x1.2389c4p+7, 0x1.043422p+8, 0x1.144e5p+7, 0x1.0a0792p+8, 0x1.144e5p+7);
b.quadTo(0x1.260864p+8, 0x1.2abedcp+7, 0x1.2d2d9ap+8, 0x1.3afcecp+7);
b.cubicTo(0x1.2d2d9ap+8, 0x1.3afcecp+7, 0x1.2db726p+8, 0x1.3d8f22p+7, 0x1.2e2d44p+8, 0x1.3d8f22p+7);
b.quadTo(0x1.3147b4p+8, 0x1.498bfep+7, 0x1.308fbp+8, 0x1.55141p+7);
b.quadTo(0x1.2fb734p+8, 0x1.62a53cp+7, 0x1.2a14c2p+8, 0x1.6f5854p+7);
b.cubicTo(0x1.2a14c2p+8, 0x1.6f5854p+7, 0x1.298b5cp+8, 0x1.71a166p+7, 0x1.28ef9p+8, 0x1.71a166p+7);
b.quadTo(0x1.1d2afcp+8, 0x1.81d2f2p+7, 0x1.0be42ep+8, 0x1.772d9ap+7);
b.cubicTo(0x1.0be42ep+8, 0x1.772d9ap+7, 0x1.0b4562p+8, 0x1.764b02p+7, 0x1.0aab8p+8, 0x1.764b02p+7);
b.quadTo(0x1.fcda2ep+7, 0x1.691ebcp+7, 0x1.ed512ep+7, 0x1.592feep+7);
b.cubicTo(0x1.ed512ep+7, 0x1.592feep+7, 0x1.ec3bfap+7, 0x1.56d836p+7, 0x1.eb48e6p+7, 0x1.56d836p+7);
b.quadTo(0x1.dfbfc2p+7, 0x1.439e1ep+7, 0x1.d82762p+7, 0x1.29570cp+7);
b.cubicTo(0x1.d82762p+7, 0x1.29570cp+7, 0x1.d79cd4p+7, 0x1.258698p+7, 0x1.d75918p+7, 0x1.258698p+7);
b.quadTo(0x1.d7ad8p+7, 0x1.061d7ep+7, 0x1.e568bp+7, 0x1.b97e2cp+6);
b.cubicTo(0x1.e568bp+7, 0x1.b97e2cp+6, 0x1.e8e2ep+7, 0x1.992cfep+6, 0x1.f436fcp+7, 0x1.992cfep+6);
b.cubicTo(0x1.feb0eap+7, 0x1.a0216p+6, 0x1.04956cp+8, 0x1.bdbdf8p+6, 0x1.076ccp+8, 0x1.bdbdf8p+6);
b.close();
    testPathOp(reporter, left, b, kIntersect_SkPathOp, filename);
}

static void (*skipTest)(skiatest::Reporter* , const char* filename) = nullptr;
static void (*firstTest)(skiatest::Reporter* , const char* filename) = nullptr;
static void (*stopTest)(skiatest::Reporter* , const char* filename) = nullptr;

#define TEST(name) { name, #name }

static struct TestDesc tests[] = {
    TEST(pentrek7),
    TEST(pentrek6),
    TEST(pentrek5),
    TEST(pentrek4),
    TEST(pentrek3),
    TEST(pentrek2),
    TEST(pentrek1),
    TEST(issue12556),
};

static const size_t testCount = std::size(tests);
static bool runReverse = false;

DEF_TEST(OpsV0, reporter) {
    RunTestSet(reporter, tests, testCount, firstTest, skipTest, stopTest, runReverse);
}

void run_v0_tests() {
    skiatest::Reporter* reporter = nullptr;
    test_OpsV0(reporter);
}
