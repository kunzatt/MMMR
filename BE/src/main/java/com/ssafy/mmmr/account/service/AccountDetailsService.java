package com.ssafy.mmmr.account.service;

import java.util.ArrayList;
import java.util.Collection;

import org.springframework.security.core.GrantedAuthority;
import org.springframework.security.core.userdetails.User;
import org.springframework.security.core.userdetails.UserDetails;
import org.springframework.security.core.userdetails.UserDetailsService;
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import com.ssafy.mmmr.account.entity.AccountEntity;
import com.ssafy.mmmr.account.repository.AccountRepository;

import lombok.RequiredArgsConstructor;

@Service
@RequiredArgsConstructor
public class AccountDetailsService implements UserDetailsService {

	private final AccountRepository accountRepository;

	@Override
	public UserDetails loadUserByUsername(String email) throws UsernameNotFoundException {

		AccountEntity account = accountRepository.findByEmail(email)
			.orElseThrow(() -> {
				return new UsernameNotFoundException("사용자를 찾을 수 없습니다: " + email);
			});

		Collection<GrantedAuthority> authorities = new ArrayList<>();

		return new User(account.getEmail(), account.getPassword(), authorities);
	}
}