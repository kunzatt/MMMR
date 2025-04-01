package com.ssafy.mmmr.transportation.entity;

import java.time.LocalDateTime;

import org.springframework.data.annotation.CreatedDate;
import org.springframework.data.annotation.LastModifiedDate;
import org.springframework.data.jpa.domain.support.AuditingEntityListener;

import com.ssafy.mmmr.profiles.entity.ProfileEntity;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.EntityListeners;
import jakarta.persistence.FetchType;
import jakarta.persistence.GeneratedValue;
import jakarta.persistence.GenerationType;
import jakarta.persistence.Id;
import jakarta.persistence.JoinColumn;
import jakarta.persistence.ManyToOne;
import jakarta.persistence.Table;
import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Entity
@Table(name = "metros")
@Getter
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@EntityListeners(AuditingEntityListener.class)
public class MetroEntity {

	@Id
	@GeneratedValue(strategy = GenerationType.IDENTITY)
	private Long id;

	@ManyToOne(fetch = FetchType.LAZY)
	@JoinColumn(name = "profile_id", nullable = false)
	private ProfileEntity profile;

	@Column(name = "line", nullable = false)
	private Integer line;

	@Column(name = "station", nullable = false, length = 20)
	private String station;

	@Column(name = "direction", length = 20)
	private String direction;

	@Column(name = "deleted", nullable = false, columnDefinition = "TINYINT DEFAULT 0")
	private Boolean deleted = false;

	@CreatedDate
	@Column(name = "created_at", nullable = false, updatable = false)
	private LocalDateTime createdAt;

	@LastModifiedDate
	@Column(name = "updated_at")
	private LocalDateTime updatedAt;

	@Builder
	public MetroEntity(ProfileEntity profile, Integer line, String station, String direction) {
		this.profile = profile;
		this.line = line;
		this.station = station;
		this.direction = direction;
		this.deleted = false;
	}

	public void delete() {
		this.deleted = true;
	}

	public void restore() {
		this.deleted = false;
	}
}