package com.ssafy.mmmr.transportation.repository.impl;

import java.util.List;

import com.querydsl.jpa.impl.JPAQueryFactory;
import com.ssafy.mmmr.profiles.entity.ProfileEntity;
import com.ssafy.mmmr.transportation.entity.MetroEntity;
import com.ssafy.mmmr.transportation.entity.QMetroEntity;
import com.ssafy.mmmr.transportation.repository.MetroRepositoryCustom;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class MetroRepositoryImpl implements MetroRepositoryCustom {

	private final JPAQueryFactory queryFactory;

	@Override
	public List<MetroEntity> findActiveMetrosByProfile(ProfileEntity profile) {
		QMetroEntity metro = QMetroEntity.metroEntity;

		return queryFactory
			.selectFrom(metro)
			.where(metro.profile.eq(profile)
				.and(metro.deleted.eq(false)))
			.fetch();
	}

	@Override
	public int countActiveMetrosByProfile(ProfileEntity profile) {
		QMetroEntity metro = QMetroEntity.metroEntity;

		return (int) queryFactory
			.selectFrom(metro)
			.where(metro.profile.eq(profile)
				.and(metro.deleted.eq(false)))
			.fetchCount();
	}
}