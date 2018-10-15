#ifndef DARTPY_TEMPLATE_METAPROGRAMMING_H_
#define DARTPY_TEMPLATE_METAPROGRAMMING_H_

/// List of types.
template <class... Types>
struct typelist
{
};

/// Concatenate two typelists.
// Source: http://stackoverflow.com/a/9122122/111426
template <class S, class T>
struct typelist_cat;

template <class ...Ss, class ...Ts>
struct typelist_cat<typelist<Ss...>, typelist<Ts...>>
{
  typedef typelist<Ss..., Ts...> type;
};


/// Cartesian product of two typelists.
// Source: http://stackoverflow.com/a/9122122/111426
template <class S, class T> struct typelist_product;

template <class S, class ...Ss, class... Ts>
struct typelist_product<typelist<S, Ss...>, typelist<Ts...>>
{
  // the cartesian product of {S} and {Ts...}
  // is a list of pairs -- here: a typelist of 2-element typelists
  typedef typelist<typelist<S, Ts>...> S_cross_Ts;

  // the cartesian product of {Ss...} and {Ts...} (computed recursively)
  typedef typename typelist_product<typelist<Ss...>, typelist<Ts...>>::type Ss_cross_Ts;

  // concatenate both products
  typedef typename typelist_cat<S_cross_Ts, Ss_cross_Ts>::type type;
};

template <typename ...Ts>
struct typelist_product<typelist<>, typelist<Ts...>>
{
  typedef typelist<> type;
};


#endif // ifndef DARTPY_TEMPLATE_METAPROGRAMMING_H_
