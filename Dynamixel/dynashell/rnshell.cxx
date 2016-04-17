// 
// rnsh (dynashell for one).
//
// This is a slow, background, low priority effort to have a simple, but
// capable shell for RoadNarrows supported products.
//
// Base Features:
//  assignment
//  conditional testing
//  iteration
//  variable management
//  built-in commands (e.g. exit, help, log, quit, wait)
//  plug-in specific application commands
//  plug-in special variable ($) support
//  botsense support
//
ShExec()
{
  ShLang *pStmt;
  int     rc;

  while( true )
  {
    rc = -DYNA_ECODE_PARSE;

    pStmt = ShParse(session);

    if( pStmt != NULL )
    {
      rc = pStmt->Eval();
      delete pStmt;
    }

    if( !bIsIteractive (rc != DYNA_OK) )
    {
      ShExit(session)
    }
  }
}

void ShError(const char *sFmt, ...)
{
}

enum ShLangToken
{
  TokenNone,

  // non-terminals
  TokenCmdStmt,
  TokenAssignStmt,
  TokenIfStmt,
  TokenElifStmt,
  TokenElseStmt,
  TokenWhileStmt,
  TokenBlockStmt,
  TokenExpr,
  TokenAddExpr,
  TokenMulExpr,
  TokenPrimaryExpr,

  // arithmetic binary operator terminals
  TokenBinOpAdd,
  TokenBinOpSub,
  TokenBinOpMul,
  TokenBinOpDiv,
  TokenBinOpMod,

  // boolean binary operator terminals
  TokenBinOpAnd,
  TokenBinOpOr,
  TokenBinOpNot,
  
  // comparative binary operator terminals
  TokenBinOpEQ,
  TokenBinOpNE,
  TokenBinOpGT,
  TokenBinOpGE,
  TokenBinOpLT,
  TokenBinOpLE,
  
  // indentifier/constant terminals
  TokenIdentifier,
  TokenBoolean,
  TokenInteger,
  TokenReal,
  TokenString
};

enum ShDataType
{
  TUndef,
  TBool   = TokenBoolean,
  TInt    = TokenInteger,
  TReal   = TokenReal,
  TString = TokenString
};

// -----------------------------------------------------------------------------
class ShVal
{
public:
  ShVal(ShDataType eObjType, const char *sObjName)
  {
    m_nStatus     = DYNA_OK;
    m_eObjType    = eObjType;
    m_sObjName    = NEWSTR(sObjName);
  }

  virtual ~ShVal()
  {
    DELSTR(m_sObjName);
  }

  static ShVal *New(bool val);
  static ShVal *New(int val);
  static ShVal *New(double val);
  static ShVal *New(const char *val);

  ShDataType GetObjType() const
  {
    return m_eObjType;
  }

  const char *GetObjName() const
  {
    return m_sObjName;
  }

  int GetStatus() const
  {
    return m_nStatus;
  }

  int SetStatus(int nStatus)
  {
    if( nStatus > 0 )
    {
      nStatus = -nStatus;
    }
    m_nStatus = nStatus;
    return m_nStatus;
  }

  void ClearStatus()
  {
    m_nStatus = DYNA_OK;
  }

  virtual bool        ToBool() const = 0;
  virtual int         ToInt() const = 0;
  virtual double      ToReal() const = 0;
  virtual const char *ToString() const = 0;

  virtual ShVal *Add(const ShVal *pRhs) = 0;
  virtual ShVal *Sub(const ShVal *pRhs) = 0;
  virtual ShVal *Mul(const ShVal *pRhs) = 0;
  virtual ShVal *Div(const ShVal *pRhs) = 0;
  virtual ShVal *Mod(const ShVal *pRhs) = 0;

  virtual bool IsEQ(const ShVal *pRhs) = 0;
  virtual bool IsNotEQ(const ShVal *pRhs) = 0;
  virtual bool IsGT(const ShVal *pRhs) = 0;
  virtual bool IsGE(const ShVal *pRhs) = 0;
  virtual bool IsLT(const ShVal *pRhs) = 0;
  virtual bool IsLE(const ShVal *pRhs) = 0;

  virtual ShVal *Dup(const ShVal *pRhs) = 0;

protected:
  ShDataType    m_eObjType;
  const char   *m_sObjName;
  int           m_nStatus;
};

// -----------------------------------------------------------------------------
class ShValInt : public ShVal
{
public:
  ShValInt(int val = 0) : ShVal(TInt, "int")
  {
    m_val = val;
  }

  virtual bool ToBool() const
  {
    return m_val != 0? true: false;
  }

  virtual int ToInt() const
  {
    return m_val;
  }

  virtual double ToReal() const
  {
    return (double)m_val;
  }

  virtual const char *ToString() const
  {
    return NULL; // TODO
  }

  virtual ShValInt &operator=(const ShValInt &rhs)
  {
    if( this != &rhs )
    {
      copy(&rhs);
    }
    return *this;
  }

  virtual ShVal *Add(const ShVal *pRhs);

protected:
  int   m_val;

  virtual void copy(const ShVal *pOrig)
  {
    m_val = pOrig->ToInt();
  }
};


// -----------------------------------------------------------------------------
class ShValReal : public ShVal
{
public:
  ShValReal(double val = 0.0) : ShVal(TReal, "real")
  {
    m_val = val;
  }

  virtual bool ToBool() const
  {
    return m_val != 0.0? true: false;
  }

  virtual int ToInt() const
  {
    return (int)m_val;
  }

  virtual double ToReal() const
  {
    return m_val;
  }

  virtual const char *ToString() const
  {
    return NULL; // TODO
  }

  ShValReal &operator=(const ShValReal &rhs)
  {
    if( this != &rhs )
    {
      copy(&rhs);
    }
    return *this;
  }

  virtual ShVal *Add(const ShVal *pRhs);

protected:
  int   m_val;

  virtual void copy(const ShVal *pOrig)
  {
    m_val = pOrig->ToReal();
  }
};

ShVal *ShValInt::Add(const ShVal *pRhs)
{
  ShVal *pVal;

  switch( pRhs->GetObjType() )
  {
    case TBool:
    case TInt:
      pVal = new ShValInt(m_val + pRhs->ToInt());
      break;
    case TReal:
      pVal = new ShValReal(ToReal() + pRhs->ToReal());
      break;
    case TString:
      ShError("Error: Unsupported operand types for 'int' and 'str'.");
      pVal = new ShValInt();
      pVal->SetStatus(DYNA_ECODE_PARSE);
      break;
    case TUndef:
    default:
      ShError("Error: Undefined R.H.S.");
      pVal = new ShValInt();
      pVal->SetStatus(DYNA_ECODE_PARSE);
      break;
  }

  return pVal;
}

ShVal *ShValReal::Add(const ShVal *pRhs)
{
  ShVal *pVal;

  switch( pRhs->GetObjType() )
  {
    case TBool:
    case TInt:
    case TReal:
      pVal = new ShValReal(m_val + pRhs->ToReal());
      break;
    case TString:
      ShError("Error: Unsupported operand types for 'real' and 'string'.");
      pVal = new ShValReal();
      pVal->SetStatus(DYNA_ECODE_PARSE);
      break;
    case TUndef:
    default:
      ShError("Error: Undefined R.H.S.");
      pVal = new ShValReal();
      pVal->SetStatus(DYNA_ECODE_PARSE);
      break;
  }

  return pVal;
}

// abstract base class
class ShLang
{
public:
  ShLang(ShLangToken eObjType, const char *sObjName)
  {
    m_eObjType  = eType;
    m_sObjName  = NEWSTR(sObjName);
    m_pResult   = NULL;
  }

  virtual ~ShLang()
  {
    DELSTR(m_sObjName);
  }

  const ShVal *GetResult()
  {
    return m_pResult;
  }

  int Eval() = 0;
  int Print() = 0;

prottected:
  ShLangToken  m_eObjType;
  const char  *m_sObjName;
  ShVal       *m_pResult;
};


class ShLangCmdStmt : public ShLang
{
  ShLangCmdStmt(const char *sCmdName, vecArg) : ShLang(TokenCmdStmt, "command");

  int Eval()
  {
    char *argv[m_argc];

    argv[0] = m_sCmdName;

    for(i=1, rc=DYNA_OK; i<m_argc; ++i)
    {
      if( (rc = m_vecArg[i]->Eval()) == DYNA_OK )
      {
        argv[i] = m_pArg->GetResult()->ToString();
      }
    }

    if( rc == DYNA_OK )
    {
      rc = Exec(session, argc, argv);
    }

    for(i=1; i<m_argc; ++i)
    {
      delete[] argv[i];
    }

    return m_result.setstatus(rc);
  }
};

class ShLangAssignStmt : public ShLang
{
  ShLangAssignStmt(const char *sIdentifier, ShLangExpr *pExpr)
        : ShLang(TokenAssignStmt, "assignment");

  int Eval()
  {
    if( (rc = m_pExpr->Eval()) == DYNA_OK )
    {
      rc = ShHashSet(m_sIdentifier, m_pExpr->GetResult());
    }
    return m_result.setstatus(rc);
  }
};

class ShLangIfStmt : public ShLang
{
  ShLangIfStmt(ShLangCondExpr *pCond, ShLangBlockStmt *pBlockStmt)
      : ShLang(TokenIfStmt, "if");

  int Eval()
  {
    if( ((rc = m_pCond->Eval()) == DYNA_OK) && 
        (m_pCond->GetResult()->ToBool() == true) )
    {
      rc = m_pBlockStmt->Eval();
    }
    return m_result.setstatus(rc);
  }
};

// same for elif

class ShLangElseStmt : public ShLang
{
  ShLangElseStmt(ShLangBlockStmt *pBlockStmt) : ShLang(TokenElseStmt, "else");

  int Eval()
  {
    rc = m_pBlockStmt->Eval();
    return m_result.setstatus(rc);
  }
};

class ShLangWhileStmt : public ShLang
{
  ShLangWhileStmt(ShLangCondExpr *pCond, ShLangBlockStmt *pBlockStmt)
      : ShLang(TokenWhileStmt, "while");

  int Eval()
  {
    int rc;

    while( ((rc = m_pCond->Eval()) == DYNA_OK) && 
           (m_pCond->GetResult()->ToBool() == true) )
    {
      if( (rc = m_pBlockStmt->Eval()) != DYNA_OK )
      {
        break;
      }
    }
    return rc;
  }
};

class ShLangBlockStmt : public ShLang
{
  ShLangBlockStmt(vecStmts) : ShLang(TokenBlockStmt, "block");

  int Eval()
  {
    int rc;

    for(i=0; i<m_vecStmts.size(); ++i)
    {
      if( (rc = m_vecStmts[i]->Eval()) != DYNA_OK )
      {
        break;
      }
    }
    return m_result.setstatus(rc);
  }
};

class ShLangExpr : public ShLang
{
public:
  ShLangExpr(ShLangExprArithExpr *p) : ShLang(TokenArithExpr, "arith-expr");
  ShLangExpr(ShLangExprCondExpr *q) : ShLang(TokenCondExpr, "cond-expr");

  int Eval()
  {
    m_result.clear()

    if( (rc = m_pAddExpr->Eval()) == DYNA_OK )
    {
      m_result = m_pAddExpr->GetResult();
    }
    return m_result.setstatus(rc);
  }

protected:
  ShLangAddExpr  *m_pLhs;
}

class ShLangAddExpr : public ShLang
{
public:
  ShLangAddExpr(ShLangAddExpr *pLhsExpr, ShBinOp eOp, ShLangMulExpr *pRhsExpr)
      : ShLang(TokenAddExpr, "add-expr")
  {
    m_pLhsExpr  = pLhsExpr;
    m_eOp       = eOp;
    m_pRhsExpr  = pRhsExpr;
    m_pResult   = NULL;
  }

  ShLangAddExpr(ShLangMulExpr *pMulExpr) : ShLang(TokenMulExpr, "add-expr")
  {
    m_pLhsExpr  = NULL;
    m_eOp       = ShBinOpUnary;
    m_pRhsExpr  = pMulExpr;
    m_pResult   = NULL;
  }

  virtual ~ShLangAddExpr()
  {
    DELOBJ(m_pLhs);
    DELOBJ(m_pRhs);
    DELOBJ(m_pResult);
  }

  int Eval()
  {
    int       rc;

    DELOBJ(m_pResult);

    if( (rc = pRhs->Eval()) != DYNA_OK )
    {
      return rc;
    }
    else if( (pLhs != NULL) && (rc = pLhs->Eval()) != DYNA_OK )
    {
      return rc;
    }

    switch( m_eOp )
    {
      case ShBinOpPlus:
        m_pResult = m_pLhs->GetResult()->Add(m_pRhs);
        break;
      case ShBinOpMinus:
        m_pResult = m_pLhs->GetResult()->Sub(m_pRhs);
        break;
      case ShBinOpUnary:
        m_pResult = m_pRhs->GetResult()->Dup();
    }

    return m_pResult->GetStatus();
  }

protected:
  ShLangAddExpr  *m_pLhs;
  ShBinOp_T       eOp;
  ShLangMulExpr  *m_pRhs;
};

class ShLangMulExpr : public ShLang
{
public:
  ShLangMulExpr(ShLangMulExpr *pLhsExpr,
                ShBinOp eOp,
                ShLangPrimaryExpr *pRhsExpr) : ShLang(TokenMulExpr, "mul-expr")
  {
    m_pLhsExpr  = pLhsExpr;
    m_eOp       = eOp;
    m_pRhsExpr  = pRhsExpr;
    m_pResult   = NULL;
  }

  ShLangMulExpr(ShLangMulExpr *pPrimaryExpr) : ShLang(TokenMulExpr, "mul-expr")
  {
    m_pLhsExpr  = NULL;
    m_eOp       = ShBinOpUnary;
    m_pRhsExpr  = pPrimaryExpr;
    m_pResult   = NULL;
  }

  virtual ~ShLangAddExpr()
  {
    DELOBJ(m_pLhs);
    DELOBJ(m_pRhs);
    DELOBJ(m_pResult);
  }

  int Eval()
  {
    int       rc;

    DELOBJ(m_pResult);

    if( (rc = pRhs->Eval()) != DYNA_OK )
    {
      return rc;
    }
    else if( (pLhs != NULL) && (rc = pLhs->Eval()) != DYNA_OK )
    {
      return rc;
    }

    switch( m_eOp )
    {
      case ShBinOpPlus:
        m_pResult = m_pLhs->Add(m_pRhs);
        break;
      case ShBinOpMinus:
        m_pResult = m_pLhs->Sub(m_pRhs);
        break;
      case ShBinOpUnary:
        m_pResult = m_pRhs->Copy();
    }

    return m_pResult->GetStatus();
  }

protected:
  ShLangMulExpr      *m_pLhs;
  ShBinOp_T           eOp;
  ShLangPrimaryExpr  *m_pRhs;
};

class ShLangPrimaryExpr : public ShLang
{
  ShLangPrimaryExpr(bool val) : ShLang(TokenPrimaryExpr, "constant")
  {
    m_eType       = CONSTANT;
    m_sIdentifier = NULL;
    m_pResult     = new ShVal::New(val);
  }

  ShLangPrimaryExpr(int val) : ShLang(TokenPrimaryExpr, "constant")
  {
    m_eType       = CONSTANT;
    m_sIdentifier = NULL;
    m_pResult     = new ShVal::New(val);
  }

  ShLangPrimaryExpr(double val) : ShLang(TokenPrimaryExpr, "constant")
  {
    m_eType       = CONSTANT;
    m_sIdentifier = NULL;
    m_pResult     = new ShVal::New(val);
  }

  ShLangPrimaryExpr(const char *val) : ShLang(TokenPrimaryExpr, "constant")
  {
    m_eType       = CONSTANT;
    m_sIdentifier = NULL;
    m_pResult     = new ShVal::New(val);
  }

  ShLangPrimaryExpr(ShLangToken eTokenIdentifier,
                    const char *val)
      : ShLang(TokenPrimaryExpr, "identifier")
  {
    m_eType       = INDENTIFIER;
    m_sIdentifier = newstr(val);
    m_pResult     = NULL;
  }

  virtual ~ShLangPrimaryExpr()
  {
    delstr(m_sIdentifier);
    delobj(m_pResult);
  }

  virtual int Eval()
  {
    switch( m_eType )
    {
      case INDENTIFER:
        delobj(m_pResult);
        m_pResult = ShHashGet(m_sIdentifier);
        break;

      // constant
      default:
        break;
    }

    return m_pResult->GetStatus();
  }
};
